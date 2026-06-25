from isaacsim import SimulationApp
launch_config = {
    "width": 1920,          # 内部渲染视口（Viewport）的宽度
    "height": 1080,         # 内部渲染视口（Viewport）的高度
    "headless": False,      # 确保是图形化显示模式
}
simulation_app = SimulationApp(launch_config)

# 导入isaacsim相关包
from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.utils.extensions import enable_extension, disable_extension
from isaacsim.core.prims import Articulation, SingleArticulation
from isaacsim.sensors.physics import _sensor
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import omni
import usdrt.Sdf
from pxr import PhysxSchema, UsdLux, UsdPhysics, Gf, UsdGeom, Usd
from omni.timeline import get_timeline_interface
# from isaaclab.sensors import FrameTransformerCfg
# from omni.isaac.core.utils import prims as prim_utils
from omni.isaac.core.prims import XFormPrim, RigidPrim
from scipy.spatial.transform import Rotation as R
import omni.kit.app
import omni.usd
from omni.isaac.core.utils.rotations import quat_to_rot_matrix
from omni.isaac.core.utils.transformations import tf_matrix_from_pose

# 导入其他功能包
import os
import numpy as np
import time
import rclpy
from rclpy.parameter import Parameter
import random
from rclpy.node import Node
from std_msgs.msg import Int32, Float64MultiArray, Empty
from geometry_msgs.msg import WrenchStamped, PoseStamped
from sensor_msgs.msg import CameraInfo
from builtin_interfaces.msg import Time
# from tf2_ros import Buffer, TransformListener
# from tf2_geometry_msgs import do_transform_pose
import math
import carb


def get_hook_mesh_bounds(stage, mesh_prim_path: str):
    '''
        获取hookrod mesh的局部/世界包围盒信息
        input:
            stage: USD舞台
            mesh_prim_path(str): mesh prim路径
        output:
            dict或None:
                {
                    "local_min": np.ndarray(3,),
                    "local_max": np.ndarray(3,),
                    "world_min": np.ndarray(3,),
                    "world_max": np.ndarray(3,),
                    "size": np.ndarray(3,)
                }
        用途:
            支持动态计算offset_b与x最大侧平面4角点
    '''
    prim = stage.GetPrimAtPath(mesh_prim_path)
    if not prim.IsValid():
        print(f"[WARN] 未找到hookrod mesh prim: {mesh_prim_path}")
        return None

    mesh = UsdGeom.Mesh(prim)
    points_attr = mesh.GetPointsAttr()
    points = points_attr.Get() if points_attr else None
    if points is None or len(points) == 0:
        print(f"[WARN] hookrod mesh points为空: {mesh_prim_path}")
        return None

    pts_np = np.array(points, dtype=float).reshape(-1, 3)
    local_min = pts_np.min(axis=0)
    local_max = pts_np.max(axis=0)

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default'])
    bbox = bbox_cache.ComputeWorldBound(prim)
    bbox_range = bbox.ComputeAlignedRange()
    world_min = np.array(bbox_range.GetMin(), dtype=float)
    world_max = np.array(bbox_range.GetMax(), dtype=float)
    size = world_max - world_min
    return {
        "local_min": local_min,
        "local_max": local_max,
        "world_min": world_min,
        "world_max": world_max,
        "size": size,
    }

# 添加路径
TRAIN_ROBOT_PATH = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
USD_PATH = os.path.join(TRAIN_ROBOT_PATH, "train_robot", "assets", "whole_stage_velocity_control.usd")
# USD_PATH = os.path.join(TRAIN_ROBOT_PATH, "train_robot", "assets", "whole_stage.usd")

# 场景初始化，加载场景
open_stage(usd_path=USD_PATH)
world = World(physics_dt=1/120, rendering_dt=1/60) # use gpu: backend="torch", device="cuda"
stage = world.stage
physx_scene = PhysxSchema.PhysxSceneAPI.Get(stage, "/World/PhysicsScene")
physx_scene.CreateSolverTypeAttr().Set("PGS")# 将物理求解器设置成PGS，减小推车的上下晃动
world.reset()


# ==== 灯光控制相关设置 ====
# 是否启用环境亮度控制
ENABLE_LIGHT_CONTROL = False
# 是否让环境“忽明忽暗”，如果为 False 则只在启动时统一调整一次亮度
ENABLE_LIGHT_FLICKER = True
# 总体亮度缩放系数（在原始强度基础上乘以这个值）
BASE_INTENSITY_SCALE = 1.0
# 忽明忽暗的亮度范围（在 BASE_INTENSITY_SCALE 的基础上上下浮动）
FLICKER_MIN_SCALE = 0.3
FLICKER_MAX_SCALE = 1.5
# 忽明忽暗的周期（秒）
FLICKER_PERIOD_SEC = 5.0

# ==== control_node退出监听与自动重置 ====
ENABLE_CONTROL_NODE_WATCHDOG = True
CONTROL_NODE_NAME = "moveit_node"
CONTROL_NODE_MISS_TIMEOUT_SEC = 1.5
RESET_SETTLE_FRAMES_BEFORE = 80
RESET_SETTLE_FRAMES_AFTER = 8
RESET_DOUBLE_PASS = True
RESET_SPEED_HOLD_SEC = 2.0

light_prims = []
light_base_intensities = {}

if ENABLE_LIGHT_CONTROL:
    # 支持的具体灯光类型（不同版本 Isaac/Usd 可能有所差异）
    light_schema_types = [
        UsdLux.DomeLight,
        UsdLux.SphereLight,
        UsdLux.RectLight,
        UsdLux.DiskLight,
        UsdLux.CylinderLight,
        UsdLux.GeometryLight,
        UsdLux.DistantLight,
    ]

    # 遍历场景中所有灯光，记录初始强度
    for prim in stage.Traverse():
        for schema in light_schema_types:
            if prim.IsA(schema):
                light = schema(prim)
                intensity_attr = light.GetIntensityAttr()
                if intensity_attr:
                    base_intensity = intensity_attr.Get()
                    if base_intensity is None:
                        continue
                    path = prim.GetPath()
                    light_prims.append(light)
                    light_base_intensities[path] = base_intensity
                break

    # 如果只做一次整体亮度调整（不忽明忽暗）
    if ENABLE_LIGHT_CONTROL and not ENABLE_LIGHT_FLICKER:
        for light in light_prims:
            path = light.GetPrim().GetPath()
            base_intensity = light_base_intensities.get(path)
            if base_intensity is not None:
                light.GetIntensityAttr().Set(base_intensity * BASE_INTENSITY_SCALE)

# 记录时间用于动态亮度
start_time = time.time()
world.step(render=True) # 进行一步仿真，确保场景加载完成
timeline = get_timeline_interface()

# 获取joint_grasp关节索引
robot_view = Articulation("/World/robot_v2")
robot_view.initialize()
joint_path = "/World/robot_v2/link6/joint_grasp"
joint = UsdPhysics.Joint.Get(stage, joint_path)
body_path = joint.GetBody1Rel().GetTargets()[0] # 获取连接的刚体名称
body_name = stage.GetPrimAtPath(body_path).GetName() # 子刚体名称
joint_grasp_index = robot_view.get_link_index(body_name)
# joint_grasp连接的子物体索引，即关节受力索引

# 获取钩提杆prim
hook_rod_prim_path = "/World/single_carriage/hooka/base_link" # 摘钩A点位置(上)
hook_rod_prim = XFormPrim(hook_rod_prim_path)
hook_rod_prim.initialize()
hook_mesh_prim_path = "/World/single_carriage/hooka/base_link/visuals/base_link/mesh"

hook_mesh_bounds = get_hook_mesh_bounds(stage, hook_mesh_prim_path)
if hook_mesh_bounds is not None:
    size_x, size_y, size_z = hook_mesh_bounds["size"]
    z_min_local = float(hook_mesh_bounds["local_min"][2])
    print(f"[INFO] Hookrod真实尺寸(X,Y,Z): ({size_x:.4f}, {size_y:.4f}, {size_z:.4f})")
    print(f"[INFO] Hookrod局部z最小点: {z_min_local:.4f}")
    world_min = hook_mesh_bounds["world_min"]
    world_max = hook_mesh_bounds["world_max"]
    _rb0 = np.array([world_max[0], world_max[1], world_min[2]], dtype=float)
    _rt0 = np.array([world_max[0], world_max[1], world_max[2]], dtype=float)
    _lt0 = np.array([world_max[0], world_min[1], world_max[2]], dtype=float)
    _lb0 = np.array([world_max[0], world_min[1], world_min[2]], dtype=float)
    print(f"[INFO] Hookrod x最大侧4角点(World) rb={_rb0}, rt={_rt0}, lt={_lt0}, lb={_lb0}")
else:
    z_min_local = None

# ROS2 初始化
rclpy.init()
node = rclpy.create_node('isaac_sensor_node') # 发布力传感器数据的节点
node.set_parameters([Parameter("use_sim_time", value=True)])
print('\n[INFO] Isaac Sensor ROS2节点创建完成')
force_publisher = node.create_publisher(WrenchStamped, 'ee_contact', 10)
print('\n[INFO] 力传感器发布者创建完成: /ee_contact')
rod_pos_publisher = node.create_publisher(PoseStamped, 'hookrod_b_pos', 10)
print('\n[INFO] 钩提杆位置发布者创建完成: /hookrod_b_pos')

print('\n[INFO] 训练推车轮速订阅创建完成: /cart_wheel_speed')

''' 控制列车运行 '''
# ==== 参数初始化 ====
frequency = 60  # 仿真频率 60hz
cart_wheel_radius = 0.5  # 车轮半径（保留用于调试记录）
wheel_names = ["wl1", "wl2", "wl3", "wl4", "wr1", "wr2", "wr3", "wr4"] # 轮子名称

# ==== 列车初始化 ====
cart_path = "/World/power_cart"
cart = Articulation(cart_path)
cart.initialize()

# 启动列车
# velocities = np.array([cart_wheel_speed]*12).reshape(1, -1)
wheel_indices = [cart.get_dof_index(name) for name in wheel_names] # 轮子索引

def apply_cart_wheel_speed(wheel_speed):
    """将列车轮速(rad/s)直接下发关节速度目标。"""
    if (not world.is_playing()) or (not cart.initialized):
        return
    ws = np.array(wheel_speed, dtype=float).reshape(-1)
    if ws.size != len(wheel_indices):
        return
    velocities = ws.reshape(1, -1)
    cart.set_joint_velocity_targets(velocities=velocities, joint_indices=wheel_indices)

# 启动时显式清零一次，避免沿用历史残留速度目标
apply_cart_wheel_speed(np.zeros(8, dtype=float))

# control_node在线检测状态
control_seen_once = False
control_missing_since = None
control_reset_done = False
is_resetting = False
reset_speed_release_wall_time = 0.0


def _clear_runtime_caches():
    """清空运行期缓存，避免复位后使用到过期状态。"""
    global velocity_count, force_msg_count, hook_msg_count
    global latest_hook_rod_b_pos, latest_hook_ori
    velocity_count = 0
    force_msg_count = 0
    hook_msg_count = 0
    latest_hook_rod_b_pos = None
    latest_hook_ori = None


def _zero_motion_once():
    """下发零速并尽量清空关键关节速度目标。"""
    apply_cart_wheel_speed(np.zeros(8, dtype=float))
    try:
        if cart.initialized:
            zero_vel = np.zeros((1, len(wheel_indices)), dtype=float)
            cart.set_joint_velocity_targets(velocities=zero_vel, joint_indices=wheel_indices)
    except Exception as exc:
        print(f"[CONTROL_MONITOR][WARN] 列车零速目标下发失败: {exc}")

    try:
        if robot_view.initialized:
            dof_names = robot_view.dof_names
            if dof_names is not None and len(dof_names) > 0:
                zero_robot = np.zeros((1, len(dof_names)), dtype=float)
                robot_indices = list(range(len(dof_names)))
                robot_view.set_joint_velocity_targets(velocities=zero_robot, joint_indices=robot_indices)
    except Exception as exc:
        print(f"[CONTROL_MONITOR][WARN] 机器人关节零速目标下发失败: {exc}")


def safe_reset_scene(reason: str):
    """
    安全复位流程：
    1) 复位前消能；2) world.reset；3) 复位后稳态；4) 可选二次reset校准。
    """
    global is_resetting, reset_speed_release_wall_time
    if is_resetting:
        print(f"[CONTROL_MONITOR][RESET] 正在复位中，忽略重复触发: {reason}")
        return

    is_resetting = True
    try:
        print(f"[CONTROL_MONITOR][RESET] 开始安全复位: {reason}")

        # 复位前：持续下发零速并step消能
        for _ in range(int(RESET_SETTLE_FRAMES_BEFORE)):
            _zero_motion_once()
            world.step(render=True)

        # 第一次reset
        world.reset()

        # 复位后：继续零速稳态，避免第一帧残留目标
        for _ in range(int(RESET_SETTLE_FRAMES_AFTER)):
            _zero_motion_once()
            world.step(render=True)

        # 可选二次reset，提高回初始位姿一致性
        if RESET_DOUBLE_PASS:
            world.reset()
            for _ in range(int(RESET_SETTLE_FRAMES_AFTER)):
                _zero_motion_once()
                world.step(render=True)

        _clear_runtime_caches()
        # 复位后冻结一小段时间列车速度，避免旧控制/碰撞残留导致再次偏移
        reset_speed_release_wall_time = time.time() + float(RESET_SPEED_HOLD_SEC)
        print("[CONTROL_MONITOR][RESET] 安全复位完成。")
    except Exception as exc:
        print(f"[CONTROL_MONITOR][ERROR] 安全复位失败: {exc}")
    finally:
        is_resetting = False


def reset_sim_when_control_node_exits():
    '''
        当control_node退出后，自动执行一次仿真重置。
    '''
    try:
        safe_reset_scene("control_node_offline")
        print("[CONTROL_MONITOR][RESET] control_node离线超时，已执行安全复位。")
    except Exception as exc:
        print(f"[CONTROL_MONITOR][ERROR] 仿真重置失败: {exc}")


def check_control_node_liveness():
    '''
        通过ROS图检测control_node在线状态，离线超时后触发一次重置。
    '''
    global control_seen_once, control_missing_since, control_reset_done
    if not ENABLE_CONTROL_NODE_WATCHDOG:
        return
    try:
        node_graph = node.get_node_names_and_namespaces()
        is_online = any(name == CONTROL_NODE_NAME for name, _ in node_graph)
    except Exception as exc:
        print(f"[CONTROL_MONITOR][ERROR] 获取ROS图失败: {exc}")
        return

    now_wall = time.time()
    if is_online:
        if not control_seen_once:
            print(f"[CONTROL_MONITOR][ONLINE] 首次检测到节点: {CONTROL_NODE_NAME}")
        control_seen_once = True
        control_missing_since = None
        control_reset_done = False
        return

    if not control_seen_once:
        return

    if control_missing_since is None:
        control_missing_since = now_wall
        print(f"[CONTROL_MONITOR][OFFLINE] 节点离线，开始计时: {CONTROL_NODE_NAME}")
        return

    missing_dt = now_wall - control_missing_since
    if (missing_dt >= float(CONTROL_NODE_MISS_TIMEOUT_SEC)) and (not control_reset_done):
        print(
            f"[CONTROL_MONITOR][OFFLINE] 节点持续离线{missing_dt:.2f}s，"
            f"超过阈值{CONTROL_NODE_MISS_TIMEOUT_SEC:.2f}s。"
        )
        reset_sim_when_control_node_exits()
        control_reset_done = True


# ==== 仿真循环 ====
velocity_count = 0
force_msg_count = 0
hook_msg_count = 0
latest_hook_rod_b_pos = None
latest_hook_ori = None

try:
    while simulation_app.is_running():
        # 获取当前仿真时间
        current_time = timeline.get_current_time() 
        sec = int(current_time) # 整数s
        nanosec = int((current_time - sec) * 1e9) # 小数转为ns
        
        if rclpy.ok(): rclpy.spin_once(node, timeout_sec=0) # 处理ros回调，发送消息
        check_control_node_liveness()

        if not world.is_playing():
            # 暂停状态持续下发零速，避免恢复后沿用旧目标
            apply_cart_wheel_speed(np.zeros(8, dtype=float))
            world.step(render=True)
            continue

        if is_resetting:
            # 复位态跳过非必要发布，减少复位过程干扰
            world.step(render=True)
            continue

        # if velocity_count == 0: # 每隔1s对列车推车进行一次速度控制

        if force_msg_count == 0:
            ''' 发布ros末端6dof力传感节点 '''
            forces = None
            # if robot_view._articulation_view and robot_view._articulation_view.is_physics_handle_valid():
            if robot_view.initialized:
                forces = robot_view.get_measured_joint_forces()
            if forces is None: # 视图未准备好时，跳过此次发布
                continue
            joint_force = forces[0][joint_grasp_index]

            force_msg = WrenchStamped()
            force_msg.header.stamp = Time(sec=sec, nanosec=nanosec) # ros2 time
            force_msg.wrench.force.x = float(joint_force[0])
            force_msg.wrench.force.y = float(joint_force[1])
            force_msg.wrench.force.z = float(joint_force[2])
            force_msg.wrench.torque.x = float(joint_force[3])
            force_msg.wrench.torque.y = float(joint_force[4])
            force_msg.wrench.torque.z = float(joint_force[5])
            force_msg.header.frame_id = "tool0"
            force_publisher.publish(force_msg)

        if hook_msg_count == 0:
            hook_mesh_bounds = get_hook_mesh_bounds(stage, hook_mesh_prim_path)
            if hook_mesh_bounds is None:
                print("[WARN] 本帧跳过hookrod位置与角点发布：mesh边界不可用。")
                continue

            # --- 钩提杆位置消息 ---
            hook_pos_a, hook_ori = hook_rod_prim.get_world_pose() # world frame
            T_hook_a = tf_matrix_from_pose(hook_pos_a, hook_ori)
            # 使用mesh局部z最小点动态构造offset_b
            z_min_local = float(hook_mesh_bounds["local_min"][2])
            offset_b = np.array([0.0, 0.0, z_min_local, 1.0], dtype=float)
            hook_rod_b_pos_homogeneous = T_hook_a @ offset_b  # P_world = T @ P_local
            hook_rod_b_pos = hook_rod_b_pos_homogeneous[:3]  # 摘钩B点位置(下)
            latest_hook_rod_b_pos = hook_rod_b_pos
            latest_hook_ori = hook_ori
            rod_msg = PoseStamped()
            rod_msg.header.stamp = Time(sec=sec, nanosec=nanosec)
            rod_msg.header.frame_id = "World" # 世界坐标系
            rod_msg.pose.position.x = float(hook_rod_b_pos[0])
            rod_msg.pose.position.y = float(hook_rod_b_pos[1])
            rod_msg.pose.position.z = float(hook_rod_b_pos[2])
            rod_pos_publisher.publish(rod_msg)
            # --- 钩提杆位置消息 ---

        # 环境亮度忽明忽暗控制
        if ENABLE_LIGHT_CONTROL and ENABLE_LIGHT_FLICKER and light_prims:
            now = time.time()
            t = now - start_time
            # 生成一个在 [0, 1] 之间平滑变化的值
            phase = (t % FLICKER_PERIOD_SEC) / FLICKER_PERIOD_SEC
            smooth = 0.5 * (1.0 + math.sin(2 * math.pi * phase))
            # 映射到 [FLICKER_MIN_SCALE, FLICKER_MAX_SCALE]
            flicker_scale = FLICKER_MIN_SCALE + (FLICKER_MAX_SCALE - FLICKER_MIN_SCALE) * smooth
            total_scale = BASE_INTENSITY_SCALE * flicker_scale

            for light in light_prims:
                path = light.GetPrim().GetPath()
                base_intensity = light_base_intensities.get(path)
                if base_intensity is not None:
                    light.GetIntensityAttr().Set(base_intensity * total_scale)

        world.step(render=True)
        velocity_count = (velocity_count + 1) % (frequency / 10) # 10 hz
        force_msg_count = (force_msg_count + 1) % (frequency / 60) # 60hz
        hook_msg_count = (hook_msg_count + 1) % (frequency / 60) # 60 hz


except KeyboardInterrupt:
    print("\n[INFO] 用户中断仿真。")

except Exception as e:
    import traceback
    print(f"循环异常中断: {e}")
    traceback.print_exc()  # 打印完整的堆栈跟踪

finally:
    # 清理并关闭仿真应用
    node.destroy_node()
    rclpy.shutdown()
    # 手动关闭并重新开启扩展，强制刷新 ROS 2 内部状态
    disable_extension("omni.isaac.ros2_bridge")
    enable_extension("omni.isaac.ros2_bridge")
    simulation_app.close()
    print("\n[INFO] 仿真应用已关闭。")
