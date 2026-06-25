#!/usr/bin/env python3
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener


class MoveItNode(Node):
    def __init__(self):
        super().__init__("moveit_node")

        self.robot = MoveItPy(node_name="moveit_node")
        self.arm_group = self.robot.get_planning_component("arm")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        time.sleep(3.0)

        self.target_pose = PoseStamped()
        self.init_pose = None
        self._execute_lock = threading.Lock()

        self.goal_state_sub = self.create_subscription(
            PoseStamped,
            "/hookrod_b_pos",
            self.receive_target_pose_callback,
            10,
        )
        self.control_order_sub = self.create_subscription(
            String,
            "/control_order",
            self.execute_control_callback,
            10,
        )
        self.get_logger().info("MoveIt Node has been started.")
        self.base_link = "base_link"
        self.end_effector_link = "tool0"
        self.go_to_initial_pose()
    
    def go_to_initial_pose(self):
        '''
        移动到预设初始位姿
        '''
        self.arm_group.set_start_state_to_current_state()
        self.arm_group.set_goal_state(configuration_name="initial") # 用预设位姿名

        # 规划并执行
        self.get_logger().info("Planning to initial pose...")
        plan_result = self.arm_group.plan()

        if plan_result:
            self.get_logger().info("Planning successful, starting execution.")
            self.robot.execute("arm", plan_result.trajectory, blocking=True)
        else:
            self.get_logger().error("Planning failed! Please check the target pose or joint limits.")

    def execute_control_callback(self, msg: String):
        """
        处理MoveIt控制指令
        """
        if msg is None:
            return
        if not self._execute_lock.acquire(blocking=False):
            self.get_logger().warn("MoveIt正在执行上一条轨迹，忽略当前控制指令。")
            return

        try:
            state = msg.data.strip().lower()
            if state == "move":
                self.move_to_lift_pose()
            elif state == "lift":
                self.execute_hook_lift()
            elif state == "decouple": # 执行摘钩指令
                self.decouple_hook()
            elif state == "circle":
                self.execute_hook_circle()
        finally:
            self._execute_lock.release()

    def receive_target_pose_callback(self, msg: PoseStamped):
        """
        收到目标位姿后的回调函数
        """
        if msg is None:
            return

        p_new = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )
        p_old = np.array(
            [
                self.target_pose.pose.position.x,
                self.target_pose.pose.position.y,
                self.target_pose.pose.position.z,
            ],
            dtype=float,
        )
        if np.linalg.norm(p_new - p_old) < 1e-3:
            return
        self.target_pose = msg

    def record_current_pose(self):
        """
        记录当前末端位姿
        """
        current_tf = self.tf_buffer.lookup_transform(self.base_link, self.end_effector_link, rclpy.time.Time())
        current_pos = [
            current_tf.transform.translation.x,
            current_tf.transform.translation.y,
            current_tf.transform.translation.z,
        ]
        current_quat = [
            current_tf.transform.rotation.x,
            current_tf.transform.rotation.y,
            current_tf.transform.rotation.z,
            current_tf.transform.rotation.w,
        ]

        current_pose = self._get_posestamp_from_pose(
            frame_id=self.base_link,
            position=current_pos,
            orientation=current_quat,
        )
        self.get_logger().info(f"Successfully recorded current pose: {current_pose}")
        return current_pose

    def return_to_pose(self, pose, scaling_factor=0.8):
        """
        返回到记录位姿
        """
        if pose is None:
            self.get_logger().warn("No pose was passed in! Cannot return to pose.")
            return
        self._move_to_position(pose, scaling_factor=scaling_factor)
        self.get_logger().info(f"Returned to pose: {pose}")

    def decouple_hook(self):
        """
        执行摘钩动作，包括移动、摘钩、回位
        """
        self.init_pose = self.record_current_pose()

        self.move_to_lift_pose()
        time.sleep(3)

        self.execute_hook_lift()
        time.sleep(2)

        self.execute_hook_circle()
        time.sleep(4)
        self.get_logger().info("Decouple hook success.")

        self.return_to_pose(self.init_pose, scaling_factor=0.8)

    def move_to_lift_pose(self):
        """
        移动到准备摘钩位置
        """
        if self.target_pose.header.frame_id == "":
            self.get_logger().warn("Frame ID was empty, setting to 'World'")
            self.target_pose.header.frame_id = "World"

        transform = self.tf_buffer.lookup_transform(
            self.base_link,
            "World",
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0),
        )
        target_pose_local = PoseStamped()
        target_pose_local.pose = do_transform_pose(self.target_pose.pose, transform)
        target_pose_local.header.frame_id = self.base_link
        # target_pose_local.pose.position.x += 0.02

        # 使用固定抓取姿态，避免沿用当前末端姿态带来的对位不一致
        grasp_orientation = np.array(
            [
                [0.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        final_quat = R.from_matrix(grasp_orientation).as_quat()

        target_pose_local.pose.orientation.x = final_quat[0]
        target_pose_local.pose.orientation.y = final_quat[1]
        target_pose_local.pose.orientation.z = final_quat[2]
        target_pose_local.pose.orientation.w = final_quat[3]
        self.get_logger().info(f"Hook point B Target pose {target_pose_local}")

        self.target_pose = target_pose_local
        self._move_to_position(target_pose_local, scaling_factor=1.0)

    def execute_hook_lift(self):
        """
        执行提钩
        """
        try:
            current_tf = self.tf_buffer.lookup_transform(
                self.base_link,
                self.end_effector_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            current_pos = np.array(
                [
                    current_tf.transform.translation.x,
                    current_tf.transform.translation.y,
                    current_tf.transform.translation.z,
                ],
                dtype=float,
            )
            current_quat = np.array(
                [
                    current_tf.transform.rotation.x,
                    current_tf.transform.rotation.y,
                    current_tf.transform.rotation.z,
                    current_tf.transform.rotation.w,
                ],
                dtype=float,
            )

            z_offset = 0.2
            lift_pos = current_pos + np.array([0.0, 0.0, z_offset], dtype=float)
            target_pose = self._get_posestamp_from_pose(
                frame_id=self.base_link,
                position=lift_pos,
                orientation=current_quat,
            )
            self.get_logger().info(f"Hook Lift Target pose {target_pose.pose.position}")
            self._move_to_position(target_pose, scaling_factor=0.8)
            self.get_logger().info(f"Hook Lift Step 1: Moving up {z_offset}m")
        except Exception as e:
            self.get_logger().error(f"[Hook Lift] Error during hook lift execution: {e}")

    def execute_hook_circle(self):
        """
        旋转钩提杆
        """
        try:
            current_tf = self.tf_buffer.lookup_transform(
                self.base_link,
                self.end_effector_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            current_pos = np.array(
                [
                    current_tf.transform.translation.x,
                    current_tf.transform.translation.y,
                    current_tf.transform.translation.z,
                ],
                dtype=float,
            )
            current_quat = np.array(
                [
                    current_tf.transform.rotation.x,
                    current_tf.transform.rotation.y,
                    current_tf.transform.rotation.z,
                    current_tf.transform.rotation.w,
                ],
                dtype=float,
            )

            arc_radius = 0.2
            arc_angle = 25
            angle_rad = np.radians(arc_angle)
            y_offset = arc_radius * np.cos(angle_rad)
            z_offset = arc_radius * np.sin(angle_rad)
            final_arc_pos = current_pos + np.array([0.0, y_offset, z_offset], dtype=float)

            rot_x = R.from_euler("x", arc_angle, degrees=True)
            target_quat = (rot_x * R.from_quat(current_quat)).as_quat()

            target_pose = self._get_posestamp_from_pose(
                frame_id=self.base_link,
                position=final_arc_pos,
                orientation=target_quat,
            )
            self.get_logger().info(f"Hook Circle Target pose {target_pose.pose.position}")
            self._move_to_position(target_pose, scaling_factor=0.8)
            self.get_logger().info("Hook Circle: Hook circle motion completed successfully")
        except Exception as e:
            self.get_logger().error(f"[Hook Circle] Error during hook circle execution: {e}")

    def _get_posestamp_from_pose(self, frame_id, position, orientation):
        """
        从位置和朝向构造PoseStamped消息
        """
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        pose_msg.pose.orientation.x = float(orientation[0])
        pose_msg.pose.orientation.y = float(orientation[1])
        pose_msg.pose.orientation.z = float(orientation[2])
        pose_msg.pose.orientation.w = float(orientation[3])
        return pose_msg

    def _move_to_position(self, target_pose, scaling_factor=1.0):
        """
        MoveIt规划并执行到指定位姿
        """
        self.arm_group.set_start_state_to_current_state()
        self.arm_group.set_goal_state(pose_stamped_msg=target_pose, pose_link=self.end_effector_link)

        plan_params = PlanRequestParameters(self.robot)
        plan_params.planning_pipeline = "pilz_industrial_motion_planner"
        plan_params.planner_id = "LIN"
        plan_params.max_velocity_scaling_factor = scaling_factor
        plan_params.max_acceleration_scaling_factor = scaling_factor

        plan_result = self.arm_group.plan(plan_params)
        if plan_result:
            self.get_logger().info(
                f"Planning successful to position {target_pose.pose.position}, executing the plan."
            )
            self.robot.execute("arm", plan_result.trajectory, blocking=True)
        else:
            self.get_logger().warning(f"Planning failed for position {target_pose.pose.position}")


def main(args=None):
    rclpy.init(args=args)
    node = MoveItNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
