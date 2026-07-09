# AGENT.md - 仿真模型转换记录

## 项目概述

将基于 Isaac Sim/Isaac Lab 的仿真模型迁移到 Gazebo Harmonic + ROS2 Jazzy 环境。

原始模型包含多个组件：
- **robot**（小车+机械臂）— `train_robot/assets/urdf/robot/urdf/robot.urdf`
- **single_carriage**（列车车厢）— `train_robot/assets/urdf/single_carriage/urdf/single_carriage.urdf`
- **grasp**（抓取装置）— `train_robot/assets/urdf/grasp/urdf/grasp.urdf`
- **hump**（驼峰）— `train_robot/assets/urdf/hump/urdf/hump.urdf`
- **hook**（钩子）— `train_robot/assets/urdf/hook/urdf/hook.urdf`

目前已转换：**robot（小车+机械臂）**、**single_carriage（列车车厢）**、**grasp（抓取装置）**、**hump（驼峰）**、**hook（钩子）**。

默认展示版加载完整场景：**hump + 3 节 single_carriage + power_cart + robot（含 grasp）+ hook**。

---

## 环境信息

- OS: Ubuntu 24.04
- ROS2: Jazzy
- Gazebo: Harmonic (v8.11.0)
- 渲染引擎: Ogre2
- GPU: 无 NVIDIA GPU 驱动（nvidia-smi 失败），仅有 Mesa 软件渲染

---

## 关键文件清单

### 工作区文件（已修改/创建）

| 文件 | 说明 |
|------|------|
| `train_robot/ros2_ws/src/robot_description/urdf/robot_complete.urdf` | 完整机器人模型 URDF（小车+机械臂+传感器） |
| `train_robot/ros2_ws/src/robot_description/launch/gazebo_full.launch.py` | Gazebo 仿真启动文件 |
| `train_robot/ros2_ws/src/robot_description/config/gazebo_controllers.yaml` | 控制器参数配置 |
| `train_robot/ros2_ws/src/robot_description/worlds/empty_sensors.sdf` | 自定义 Gazebo 世界文件（含传感器系统插件） |
| `train_robot/ros2_ws/src/robot_description/setup.py` | 包安装配置（确保 worlds 目录被安装） |
| `train_robot/ros2_ws/src/gz_ros2_control/` | 从源码编译的 gz_ros2_control 插件（修复了参数传递 bug） |

### 原始资产文件（未修改）

| 文件 | 说明 |
|------|------|
| `train_robot/assets/urdf/robot/urdf/robot.urdf` | 原始机器人 URDF |
| `train_robot/assets/urdf/single_carriage/urdf/single_carriage.urdf` | 原始列车车厢 URDF（已转换为 `single_carriage_gazebo.urdf`） |
| `train_robot/assets/urdf/grasp/urdf/grasp.urdf` | 原始抓取装置 URDF |
| `train_robot/assets/urdf/hump/urdf/hump.urdf` | 原始驼峰 URDF |
| `train_robot/assets/urdf/hook/urdf/hook.urdf` | 原始钩子 URDF |
| `train_robot/assets/power_cart.usd` | 原始动力推车 USD（已导出为 `power_cart_export/power_cart.urdf`） |
| `train_robot/assets/configuration/*.usd` | Isaac Sim 配置文件 |
| `train_robot/ros2_ws/src/robot_description/meshes/` | 所有 STL 网格文件 |

---

## 修改记录

### 1. 环境搭建

- 安装 ROS2 Jazzy 基础包
- 安装 Gazebo Harmonic (`gz-harmonic`)
- 安装 ros_gz 桥接包 (`ros-jazzy-ros-gz`, `ros-jazzy-ros-gz-sim`)
- 安装 ros2_control 相关包 (`ros-jazzy-controller-manager`, `ros-jazzy-joint-state-broadcaster`, `ros-jazzy-velocity-controllers`)
- 安装 `ros-jazzy-gz-ros2-control` 和 `ros-jazzy-gz-ros2-control-demos`

### 2. 创建 `robot_complete.urdf`

基于原始 `robot.urdf` 创建，添加了以下内容：

- **ros2_control 接口**：定义硬件接口（位置、速度、力矩），配置 `gz_ros2_control/GazeboSimSystem` 插件
- **传感器定义**（在 `<gazebo>` 标签中）：
  - `camera_down`：向下相机，位于 base_link，分辨率 640x480，30Hz
  - `camera_up`：向上相机，位于 base_link，分辨率 640x480，30Hz
  - `camera_hand`：手部相机，位于 link6，分辨率 640x480，30Hz
  - `lidar_down`：向下激光雷达，位于 base_link，360度，10Hz
  - `lidar_up`：向上激光雷达，位于 base_link，360度，10Hz
- **传感器链接**：camera_down_link, camera_up_link, camera_hand_link, lidar_down_link, lidar_up_link（带惯性属性）
- **固定关节保留**：`<preserveFixedJoint>true</preserveFixedJoint>` 防止固定关节被合并
- **传感器属性**：使用 SDF 标准格式 `always_on`（非 `alwaysOn`）

### 3. 修复关节名冲突

原始 URDF 中关节名与链接名重复（wl1, wr1, wl2, wr2, f1, f2），导致 Gazebo 报错。修复方案：
- 关节名：`wl1_joint`, `wr1_joint`, `wl2_joint`, `wr2_joint`, `f1_joint`, `f2_joint`
- 链接名保持不变：`wl1`, `wr1`, `wl2`, `wr2`, `f1`, `f2`

### 4. 修复空白链接

`grasp_base_link` 和 `tool0` 缺少惯性属性，导致 Gazebo 崩溃（`std::out_of_range in FreeGroupFeatures`）。修复：添加基本惯性属性。

### 5. 创建 `gazebo_full.launch.py`

启动文件包含：
- 环境变量设置：`GZ_SIM_SYSTEM_PLUGIN_PATH`, `GZ_PLUGIN_PATH` 指向自定义 gz_ros2_control 插件
- Gazebo 仿真启动（使用 `empty_sensors.sdf` 世界文件）
- `robot_state_publisher` 节点
- `ros_gz_bridge` 桥接节点（clock, 3个相机, 2个激光雷达）
- 机器人 Spawn 节点
- 控制器 Spawner（joint_state_broadcaster, velocity_controller）
- RViz2 节点

### 6. 创建 `empty_sensors.sdf`

自定义 Gazebo 世界文件，包含：
- 物理系统插件 (`gz-sim-physics-system`)
- 用户命令系统 (`gz-sim-user-commands-system`)
- 场景广播器 (`gz-sim-scene-broadcaster-system`)
- 接触系统 (`gz-sim-contact-system`)
- **传感器系统** (`gz-sim-sensors-system`) — 关键：传感器渲染必需
- 光源和地面

### 7. 从源码编译 gz_ros2_control

原因：二进制包中控制器参数传递有 bug，无法正确解析 params 文件。

修复内容：
- 将 `gz_ros2_control_plugin.cpp` 中控制器 spawner 的参数从 `-p` 改为 `--param`
- 编译安装到 `train_robot/ros2_ws/install/gz_ros2_control/`

### 8. 传感器配置调试过程

传感器初始不发布数据，经过多次尝试：

1. **创建含传感器系统的世界文件**：默认世界文件不包含传感器系统插件
2. **修复插件名**：`gz-sim-sensor-system` → `gz-sim-sensors-system`（复数形式）
3. **修复 SDF 属性**：`alwaysOn` → `always_on`（SDF 标准格式）
4. **传感器定义位置**：从链接引用块移到关节引用块，与 `preserveFixedJoint` 合并，确保父实体存在
5. **分离材质定义**：传感器链接的材质定义为独立的 `<gazebo reference="link">` 块

**最终结果**：传感器成功初始化（Gazebo 日志确认），话题 `/camera_down`, `/camera_up`, `/camera_hand`, `/lidar_down`, `/lidar_up` 已创建。

### 9. 列车车厢模型转换

基于原始 `single_carriage.urdf` 创建 `single_carriage_gazebo.urdf`，主要修改：

- **修复 mesh 路径**：将相对路径改为绝对路径，例如：
  ```xml
  <mesh filename="/home/zhan/train_robot-main/train_robot/assets/urdf/single_carriage/meshes/base_link.STL" />
  ```
- **关节名添加后缀**：关节名添加 `_joint` 后缀避免与链接名冲突（wr1→wr1_joint, wl1→wl1_joint 等）
- **设置物理属性**：保留所有惯性、碰撞、视觉属性

### 10. 创建 `gazebo_display.launch.py`

简化版启动文件，仅加载模型和传感器，不加载控制器：

- 环境变量设置：`GZ_SIM_RESOURCE_PATH` 指向列车车厢 mesh 目录
- Gazebo 仿真启动（使用 `empty_sensors.sdf`）
- `robot_state_publisher` 节点
- `ros_gz_bridge` 时钟桥接
- 机器人 Spawn 节点（位置 x:0, y:0, z:0.5）
- 列车车厢 Spawn 节点（位置 x:3.0, y:0, z:0.65）

### 11. power_cart USD 转 URDF

使用 Isaac Sim 自带的 `isaacsim.asset.exporter.urdf` 扩展中的 USD to URDF exporter，将原始动力推车 USD 导出为：

```text
train_robot/ros2_ws/src/robot_description/urdf/power_cart_export/power_cart.urdf
train_robot/ros2_ws/src/robot_description/urdf/power_cart_export/meshes/
```

本机 exporter 的 `nvidia` namespace 会被其他 Isaac 扩展遮挡，因此使用下面的 Python 方式调用：

```bash
cd /home/zhan/train_robot-main
python3 - <<'PY'
from pathlib import Path
import nvidia
nvidia.__path__.insert(0, '/home/zhan/isaacsim/exts/isaacsim.asset.exporter.urdf/pip_prebundle/nvidia')
from nvidia.srl.from_usd.to_urdf import UsdToUrdf

input_usd = '/home/zhan/train_robot-main/train_robot/assets/power_cart.usd'
output_urdf = '/home/zhan/train_robot-main/train_robot/ros2_ws/src/robot_description/urdf/power_cart_export/power_cart.urdf'
mesh_dir = '/home/zhan/train_robot-main/train_robot/ros2_ws/src/robot_description/urdf/power_cart_export/meshes'

converter = UsdToUrdf.init_from_file(input_usd, root='/power_cart')
converter.save_to_file(
    urdf_output_path=output_urdf,
    mesh_dir=mesh_dir,
    mesh_path_prefix='meshes/',
    visualize_collision_meshes=True,
)
PY
perl -0pi -e 's#meshes/meshes/#meshes/#g' train_robot/ros2_ws/src/robot_description/urdf/power_cart_export/power_cart.urdf
```

导出后检查：

```bash
source /opt/ros/jazzy/setup.bash
check_urdf train_robot/ros2_ws/src/robot_description/urdf/power_cart_export/power_cart.urdf
```

### 12. 传感器配置优化

修复传感器定义位置问题：

- **问题**：`gz_ros2_control` 插件会跳过 fixed joints，导致传感器父节点找不到
- **解决方案**：将传感器定义从关节引用块（`<gazebo reference="camera_down_joint">`）改为链接引用块（`<gazebo reference="camera_down_link">`）
- **效果**：传感器成功初始化并启用

### 13. 机械爪-车钩虚拟接触力模型

当前不再依赖 Gazebo 网格碰撞力来表示机械爪和车钩之间的接触力。原因是车钩、缓冲器、机械爪内部结构没有进行高精度动力学建模，直接读取 Gazebo 碰撞力会失真。现在采用独立 ROS2 节点计算虚拟接触力：

```text
train_robot/ros2_ws/src/robot_description/robot_description/virtual_coupler_force_node.py
train_robot/ros2_ws/src/robot_description/launch/virtual_coupler_force.launch.py
```

模型参考开源机械臂力控/导纳控制项目的常见接口与思路：用 `geometry_msgs/WrenchStamped` 输出力，内部用带间隙的弹簧-阻尼-摩擦-迟滞模型计算接触力。

#### 13.1 接触点定义

模型定义两个虚拟点：

```text
tool_contact_point      机械爪末端接触点
coupler_contact_point   车钩受力点
```

当前参数已经将两个点在 `x/z` 方向对齐，主要沿世界坐标 `y` 方向计算接触压缩：

```text
tool_contact_offset    = [-0.3305, 0.017, 1.284]
coupler_contact_offset = [0.0, -0.652, 1.304]
contact_axis_world     = [0.0, 1.0, 0.0]
```

#### 13.2 距离与间隙

先计算两点相对向量：

```text
relative = tool_contact_point - coupler_contact_point
```

沿接触方向的距离：

```text
normal_distance = relative · n
```

横向误差：

```text
lateral_distance = || relative - normal_distance * n ||
```

如果横向误差大于 `lateral_window`，认为机械爪没有对准车钩，不计算接触力。

模型设置自由间隙：

```text
free_gap = 0.12 m
```

当：

```text
normal_distance > free_gap
```

说明还未接触，力为 0。

当：

```text
normal_distance < free_gap
```

说明进入接触区，压缩量为：

```text
compression = free_gap - normal_distance
```

#### 13.3 法向力：弹簧-阻尼模型

法向接触力采用惩罚接触模型：

```text
F_n = K * compression + D * compression_rate
```

其中：

```text
compression       压缩量
compression_rate  压缩速度
K                 等效刚度
D                 等效阻尼
```

#### 13.4 迟滞：加载/卸载参数不同

为模拟车钩缓冲器压缩和回弹时力曲线不同，模型区分加载与卸载：

```text
compression_rate >= 0  加载阶段
compression_rate <  0  卸载阶段
```

当前参数：

```text
K_load   = 8000 N/m
D_load   = 180 Ns/m
K_unload = 4500 N/m
D_unload = 90 Ns/m
```

因此压入时力更大，回弹时力更小，形成简单迟滞效果。

#### 13.5 切向摩擦

摩擦力使用平滑库仑摩擦模型：

```text
F_t = -mu * F_n * tanh(v_t / v_s)
```

其中：

```text
mu  = 0.28
v_s = 0.02 m/s
v_t = 机械爪相对车钩的切向速度
```

使用 `tanh()` 是为了避免速度接近 0 时摩擦力发生突变。

#### 13.6 最终输出

最终接触力：

```text
F_contact = F_n * n + F_t * t
```

并经过一阶低通滤波：

```text
F_filtered = alpha * F_contact + (1 - alpha) * F_previous
alpha = 0.25
```

发布话题：

```text
/virtual_coupler_force         geometry_msgs/msg/WrenchStamped
/virtual_coupler_contact_state std_msgs/msg/Float64MultiArray
```

`/virtual_coupler_contact_state` 字段含义：

```text
data[0] = compression        压缩量
data[1] = compression_rate   压缩速度
data[2] = normal_force       法向力
data[3] = lateral_distance   横向误差
data[4] = normal_distance    法向距离
data[5] = contact_flag       是否接触，1 为接触，0 为未接触
data[6] = force.x
data[7] = force.y
data[8] = force.z
```

当前测试结果中，两个虚拟接触点的横向误差已经约为 `1.2e-6 m`，说明 `x/z` 方向已对准；但 `normal_distance` 仍大于 `free_gap`，所以当前输出力为 0。后续机械爪沿 `y` 方向靠近车钩并满足 `normal_distance < free_gap` 后，模型会开始输出接触力。

### 14. 当前未解决问题

**1. 控制器加载失败**：

- `gz_ros2_control` 插件的参数传递问题，参数文件路径为空
- 日志显示：`--params-file ''`
- 原因：插件在解析 URDF 时 `<parameters>` 标签的路径没有被正确读取
- 影响：`joint_state_broadcaster` 和 `velocity_controller` 无法加载
- 当前解决方案：使用 `gazebo_display.launch.py` 跳过控制器加载

**2. 传感器无数据发布**：

- Gazebo 日志显示传感器已初始化并启用：
  ```
  Camera images for [train_robot::base_link::camera_down] advertised on [camera_down]
  Enabling camera sensor: 'train_robot::base_link::camera_down' data generation.
  ```
- 但 `gz topic -i -t /camera_down` 显示 "No publishers"
- ROS2 话题 `ros2 topic hz /camera_down` 显示 "does not appear to be published yet"

**根本原因**：无 GPU 硬件支持。`nvidia-smi` 失败，系统仅有 Mesa 软件渲染。Gazebo 的 Ogre2 渲染引擎需要 GPU 来渲染相机/激光雷达传感器的图像数据。在无 GPU 环境下，传感器系统虽然初始化成功但无法实际渲染帧。

**可能的解决方案**：
1. 在有 NVIDIA GPU 的机器上运行
2. 使用 `ogre`（Ogre1）渲染引擎替代 `ogre2`（兼容性可能更好，但仍有性能问题）
3. 使用 EGL/headless 模式运行（需要配置环境变量）
4. 使用 `LIBGL_ALWAYS_SOFTWARE=1` 强制软件渲染（性能极低，不推荐）

---

## 启动命令

```bash
# 1. 进入 ROS2 工作区
cd /home/zhan/train_robot-main/train_robot/ros2_ws

# 2. 编译
colcon build --symlink-install

# 3. 加载环境
source install/setup.bash

# 4. 推荐：启动 Gazebo 展示版（加载 hump + 3 节车厢 + power_cart + robot + hook）
ros2 launch robot_description gazebo_display.launch.py
```

`power_cart` 位于 3 节车厢后方（`y=-190.0`），当前已把推车本体材质改为红色，Gazebo 里红色车辆就是动力推车。

当前推荐的 `gazebo_display.launch.py` 是展示和调试版本，只负责加载模型，不自动启动机械臂控制程序，也不启动虚拟接触力节点。

另开终端启动虚拟接触力模型：

```bash
cd /home/zhan/train_robot-main/train_robot/ros2_ws
source install/setup.bash
ros2 launch robot_description virtual_coupler_force.launch.py
```

查看虚拟接触力模型输出：

```bash
ros2 topic echo /virtual_coupler_force
ros2 topic echo /virtual_coupler_contact_state
```

`hump` 是很大的驼峰/坡道模型（约 20m x 400m）。如果只想调试车厢、机器人和 hook 的局部位置，可以临时关闭 hump：

```bash
cd /home/zhan/train_robot-main/train_robot/ros2_ws
source install/setup.bash
ros2 launch robot_description gazebo_display.launch.py load_hump:=false
```

如需启动旧的带 `ros2_control` 控制器配置的完整版本：

```bash
cd /home/zhan/train_robot-main/train_robot/ros2_ws
source install/setup.bash
ros2 launch robot_description gazebo_full.launch.py
```

注意：当前主要调试请优先使用 `gazebo_display.launch.py`。`gazebo_full.launch.py` 仍保留旧的 `ros2_control` 配置，可能出现 controller 加载问题。

完整版本临时关闭 hump：

```bash
cd /home/zhan/train_robot-main/train_robot/ros2_ws
source install/setup.bash
ros2 launch robot_description gazebo_full.launch.py load_hump:=false
```

如果 Gazebo 里出现模型重复，先确认并停止旧仿真进程，再重新启动：

```bash
ps -eo pid,cmd | grep -E "ros2 launch|gz sim|robot_state_publisher|parameter_bridge"
# 在原启动终端按 Ctrl-C 停止；如果进程残留，再按 PID kill
kill <PID>
```

查看传感器话题：

```bash
ros2 topic list | grep -E "camera|lidar"
ros2 topic echo /camera_down
```

查看虚拟车钩接触力：

```bash
ros2 topic echo /virtual_coupler_force
ros2 topic echo /virtual_coupler_contact_state
ros2 bag record /virtual_coupler_force /virtual_coupler_contact_state /clock
```

---

## 待办事项

- [] 转换列车车厢模型 (`single_carriage.urdf`) 到 Gazebo ✓
- [ ] 决定后续是否继续修复 `ros2_control` 控制链；当前展示版仅加载模型，不启动控制程序
- [ ] 解决传感器无 GPU 渲染问题（需要 GPU 或 headless 渲染方案）
- [] 转换并加载其他组件（grasp, hump, hook）
- [ ] 验证所有传感器数据通过 ros_gz_bridge 正常传输
- [ ] 添加 IMU 传感器（如果需要）
- [ ] 配置更复杂的 Gazebo 世界环境

---

## 参考链接

- Gazebo Harmonic 文档: https://gazebosim.org/docs/harmonic
- ROS2 Jazzy 文档: https://docs.ros.org/en/jazzy/
- ros_gz 桥接文档: https://docs.ros.org/en/jazzy/p/ros_gz/
- gz_ros2_control 文档: https://github.com/ros-controls/gz_ros2_control
- SDF 规范: http://sdformat.org/spec
