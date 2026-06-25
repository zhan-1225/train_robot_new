# AGENT.md - 仿真模型转换记录

## 项目概述

将基于 Isaac Sim/Isaac Lab 的仿真模型迁移到 Gazebo Harmonic + ROS2 Jazzy 环境。

原始模型包含多个组件：
- **robot**（小车+机械臂）— `train_robot/assets/urdf/robot/urdf/robot.urdf`
- **single_carriage**（列车车厢）— `train_robot/assets/urdf/single_carriage/urdf/single_carriage.urdf`
- **grasp**（抓取装置）— `train_robot/assets/urdf/grasp/urdf/grasp.urdf`
- **hump**（驼峰）— `train_robot/assets/urdf/hump/urdf/hump.urdf`
- **hook**（钩子）— `train_robot/assets/urdf/hook/urdf/hook.urdf`

目前已转换：**robot（小车+机械臂）**、**single_carriage（列车车厢）**。其他组件（grasp, hump, hook）尚未转换。

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

### 11. 传感器配置优化

修复传感器定义位置问题：

- **问题**：`gz_ros2_control` 插件会跳过 fixed joints，导致传感器父节点找不到
- **解决方案**：将传感器定义从关节引用块（`<gazebo reference="camera_down_joint">`）改为链接引用块（`<gazebo reference="camera_down_link">`）
- **效果**：传感器成功初始化并启用

### 12. 当前未解决问题

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
# 1. 编译
cd /home/zhan/train_robot-main/train_robot/ros2_ws
colcon build --symlink-install

# 2. 启动仿真
source install/setup.bash
ros2 launch robot_description gazebo_full.launch.py

# 3. 查看传感器话题
ros2 topic list | grep -E "camera|lidar"
ros2 topic echo /camera_down
```

---

## 待办事项

- [x] 转换列车车厢模型 (`single_carriage.urdf`) 到 Gazebo ✓
- [ ] 解决控制器加载失败问题（gz_ros2_control 参数传递 bug）
- [ ] 解决传感器无 GPU 渲染问题（需要 GPU 或 headless 渲染方案）
- [ ] 转换其他组件（grasp, hump, hook）
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