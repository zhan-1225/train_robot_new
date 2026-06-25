#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, PointCloud2
from rosgraph_msgs.msg import Clock
import numpy as np
from cv_bridge import CvBridge
import cv2
from sensor_msgs_py import point_cloud2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import datetime
import time
import os
from ultralytics import YOLO



#导入信号处理函数
from .train_images import train_visualize_images


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        cv2.ocl.setUseOpenCL(True)
        self.bridge = CvBridge()

        # 保存图片相关配置
        self.save_images = False
        # 只需要配置一个根目录，程序会在下面自动创建时间戳文件夹和三个相机子文件夹
        self.save_base_dir = "/home/aiden/桌面/hook_images_data"  # 修改为你想要的保存根目录
        # 每隔多少秒保存一次（每个相机单独计时），例如 0.5 表示每 0.5 秒保存一张
        self.save_interval_sec = 0.3
        # 记录三个相机上一次保存时间
        self.last_save_time = {0: None, 2: None, 4: None}

        if self.save_images:
            session_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.save_session_dir = os.path.join(self.save_base_dir, session_timestamp)
            self.save_dir_down = os.path.join(self.save_session_dir, "down_rgb")
            self.save_dir_hand = os.path.join(self.save_session_dir, "hand_rgb")
            self.save_dir_up = os.path.join(self.save_session_dir, "up_rgb")
            os.makedirs(self.save_dir_down, exist_ok=True)
            os.makedirs(self.save_dir_hand, exist_ok=True)
            os.makedirs(self.save_dir_up, exist_ok=True)

        #yolo模型
        self.yolo_model = YOLO('/home/aiden/project/research_group/train_robot/runs/obb/custom_obb_train6/weights/best.pt')

        # 相关设置，每几个节拍更新一次数据
        self.update_robot_state = 1
        self.update_picture = 1
        self.update_lidar = 1
        self.counter_robot_state = 0
        self.counter_picture = 0
        self.counter_lidar = 0

        # ===============================订阅消息=====================================
        # 注意：所有话题，数据的更新频率都是60HHz，与仿真频率相同，如果需要降低定于频率，可以在在回调函数中添加计时器
        self.start_time = None
        self.current_time = None
        self.car_speed = None
        self.arm_velocity = None
        self.arm_position = None
        self.arm_torque = None
        #图像
        self.picture_rgb_down = None
        self.picture_dp_down = None
        self.picture_rgb_up = None
        self.picture_dp_up = None
        self.picture_rgb_hand = None
        self.picture_dp_hand = None
        #原始深度图
        self.depth_down = None
        self.depth_up = None
        self.depth_hand = None
        #激光雷达点云
        self.lidar_down = None
        self.lidar_up = None


        # 订阅时钟话题
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',  # 确保Isaac Sim发布的时钟话题名称匹配
            self.clock_callback,
            10
        )
        # 订阅关节状态话题
        self.robot_state_sub = self.create_subscription(
            JointState,
            '/joint_states_sub',
            self.robot_state_sub_callback,
            10
        )
        # 订阅相机话题
        picture_topics = ['camera_down_rgb',            #下方相机彩色图像
                          'camera_down_dp',             #下方相机深度图
                          'camera_hand_rgb',            #上方相机彩色图像
                          'camera_hand_dp',             #上方相机深度图像
                          'camera_up_rgb',              #上方相机彩色图像
                          'camera_up_dp',               #上方相机深度图像
                          ]
        self.pictures_sub = []
        for i in range(len(picture_topics)):
            self.pictures_sub.append(
                self.create_subscription(
                    Image,
                    f'{picture_topics[i]}',  # 根据实际话题名修改
                    lambda msg, idx=i: self.pictures_callback(msg, idx),
                    10
                )
            )

        # 订阅激光雷达话题
        sub_lidar = False
        if sub_lidar:
            lidar_topics = ['lidar_down', 'lidar_up']
            self.lidar_sub = []
            for i in range(len(lidar_topics)):
                self.lidar_sub.append(
                    self.create_subscription(
                        PointCloud2,
                        f'{lidar_topics[i]}',  # 根据实际话题名修改
                        lambda msg, idx=i: self.lidar_callback(msg, idx),
                        10
                    )
                )

        #==============================发布消息=====================================
        self.robot_state_pub = self.create_publisher(JointState, '/robot_joint_state', 10)

        # 控制周期定时器
        self.timer = self.create_timer(0.1, self.control_cycle)

        #打印信息
        # self.info_print = self.create_timer(2, self.info_print)

        # 图像可视化
        self.image_show = self.create_timer(0.2, self.visualize_images)

        #3D点云可视化
        # self.pointclould_show = self.create_timer(1, self.visualize_pointcloud)
    
    def clock_callback(self, msg):
        """时钟回调函数，更新当前时间戳"""
        if msg is None:
            return
        self.current_time = msg.clock
        if not self.start_time:
            self.start_time = self.current_time
        return
    
    def robot_state_sub_callback(self, msg):
        """关节状态回调函数，处理接收到的关节状态数据"""
        if msg is None:
            return
        if self.counter_robot_state != 0:
            self.counter_robot_state += 1
            self.counter_robot_state = self.counter_robot_state % self.update_robot_state
            return
        self.counter_robot_state += 1
        self.counter_robot_state = self.counter_robot_state % self.update_robot_state
        
        joints = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'wl1', 'wr1', 'wl2', 'wr2']
        index = []
        for joint in joints:
            index.append(msg.name.index(joint))
        self.car_speed = [msg.velocity[i] for i in index[-4:]]
        self.arm_velocity = [msg.velocity[i] for i in index[:6]]
        self.arm_position = [msg.position[i] for i in index[:6]]
        self.arm_torque = [msg.effort[i] for i in index[:6]]
        return
    
    def pictures_callback(self, msg, idx):
        """相机回调函数，处理接收到的相机图像数据"""
        if msg is None:
            return
        if self.counter_picture != 0:
            self.counter_picture += 1
            self.counter_picture = self.counter_picture % self.update_picture
            return
        self.counter_picture += 1
        self.counter_picture = self.counter_picture % self.update_picture

        MAX_DIS = 5.0
        MIN_DIS = 0.1
        if idx == 0:
            self.picture_rgb_down = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        elif idx == 1:
            depth_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_down = depth_data
            depth_normalized = np.clip((depth_data - MIN_DIS) * (255 / MAX_DIS), 0, 255).astype(np.uint8)
            self.picture_dp_down = depth_normalized
        elif idx == 2:
            self.picture_rgb_hand = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        elif idx == 3:
            depth_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_hand = depth_data
            depth_normalized = np.clip((depth_data - MIN_DIS) * (255 / MAX_DIS), 0, 255).astype(np.uint8)
            self.picture_dp_hand = depth_normalized
        elif idx == 4:
            self.picture_rgb_up = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        elif idx == 5:
            depth_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_up = depth_data
            depth_normalized = np.clip((depth_data - MIN_DIS) * (255 / MAX_DIS), 0, 255).astype(np.uint8)
            self.picture_dp_up = depth_normalized

        # 如果开启保存图片，则按设定的时间间隔保存到文件夹
        if self.save_images and idx in self.last_save_time:
            now = time.time()
            last = self.last_save_time[idx]
            if last is None or (now - last) >= self.save_interval_sec:
                self.last_save_time[idx] = now
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                if idx == 0 and self.picture_rgb_down is not None:
                    filename = os.path.join(self.save_dir_down, f"down_rgb_{timestamp}.jpg")
                    cv2.imwrite(filename, self.picture_rgb_down, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                elif idx == 2 and self.picture_rgb_hand is not None:
                    filename = os.path.join(self.save_dir_hand, f"hand_rgb_{timestamp}.jpg")
                    cv2.imwrite(filename, self.picture_rgb_hand, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                elif idx == 4 and self.picture_rgb_up is not None:
                    filename = os.path.join(self.save_dir_up, f"up_rgb_{timestamp}.jpg")
                    cv2.imwrite(filename, self.picture_rgb_up, [int(cv2.IMWRITE_JPEG_QUALITY), 90])

        return
    
    def lidar_callback(self, msg, idx):
        """激光雷达回调函数，处理接收到的激光雷达数据"""
        if msg is None:
            return
        if self.counter_lidar != 0:
            self.counter_lidar += 1
            self.counter_lidar = self.counter_lidar % self.update_lidar
            return
        self.counter_lidar += 1
        self.counter_lidar = self.counter_lidar % self.update_lidar
        if idx == 0:
            self.lidar_down = point_cloud2.read_points_numpy(msg)
        elif idx == 1:
            self.lidar_up = point_cloud2.read_points_numpy(msg)
        return
    
    def visualize_images(self):
        train_visualize_images(
            self.picture_rgb_down,
            self.picture_dp_down,
            self.picture_rgb_hand,
            self.picture_dp_hand,
            self.picture_rgb_up,
            self.picture_dp_up,
            self.yolo_model
        )
    
    def visualize_pointcloud(self):
        if self.depth_down is not None and self.picture_rgb_down is not None:
            try:
                from .train_point_cloud import dep2pcd
            except ImportError as exc:
                self.get_logger().error(f"Point cloud visualization is unavailable: {exc}")
                return
            dep2pcd(self.depth_down, self.picture_rgb_down)


    def set_car_speed(self, angle_speed):
        '''
        function: 设置小车四个车轮的速度(wl1,wlr1,wl2,wr2)
        parameters:
            angle_speed: 推车车轮的角速度(rad/s)
        '''
        wheel_speed_msg = JointState()
        wheel_speed_msg.name = ['wl1', 'wr1', 'wl2', 'wr2']
        wheel_speed_msg.velocity = [float(v) for v in angle_speed]
        self.robot_state_pub.publish(wheel_speed_msg)
        return
    
    def set_arm_speed(self, angle_speed):
        '''
        function: 速控模式下控制机械臂，设置机械臂六个关节的速度(j1, j2, j3, j4, j5, j6)
        parameters:
            angle_speed: 机械臂关节的角速度(rad/s)
        '''
        arm_state_msg = JointState()
        arm_state_msg.name = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        arm_state_msg.velocity = [float(v) for v in angle_speed]
        self.robot_state_pub.publish(arm_state_msg)
        return
    
    def set_arm_position(self, angle_position):
        '''
        function: 位置模式下控制机械臂，设置机械臂六个关节的位置(j1, j2, j3, j4, j5, j6)
        parameters:
            angle_position: 机械臂关节的位置(rad)
        '''
        arm_state_msg = JointState()    
        arm_state_msg.name = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        arm_state_msg.position = [float(v) for v in angle_position]
        self.robot_state_pub.publish(arm_state_msg)
        return
    
    def set_arm_torque(self, torque):
        '''
        function: 力矩模式下控制机械臂，设置机械臂六个关节的力矩(j1, j2, j3, j4, j5, j6)
        parameters:
            torque: 机械臂关节的力矩(Nm)
        '''
        arm_state_msg = JointState()
        arm_state_msg.name = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        arm_state_msg.effort = [float(v) for v in torque]
        self.robot_state_pub.publish(arm_state_msg)
        return

    def control_cycle(self):
        """核心控制逻辑"""
        '''
        if None in (self.current_lidar, self.current_image):
            return
        
        '''
        car_apeed = 0.0
        self.set_car_speed([car_apeed, car_apeed, car_apeed, car_apeed])
        # self.set_arm_position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        return

    def info_print(self):
        # print(f"Time updated: {self.current_time.sec}.{self.current_time.nanosec}")
        # print(f"Car speed: {self.car_speed}")
        # print(f"Arm velocity: {self.arm_velocity}")
        # print(f"Arm position: {self.arm_position}")
        # print(f"Arm torque: {self.arm_torque}")
        pass
    

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
