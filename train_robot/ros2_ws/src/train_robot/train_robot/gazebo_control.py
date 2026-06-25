#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, PointCloud2
from rosgraph_msgs.msg import Clock
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from cv_bridge import CvBridge
import cv2
from sensor_msgs_py import point_cloud2
import datetime
import time
import os
from ultralytics import YOLO


from .train_images import train_visualize_images


class GazeboRobotControlNode(Node):
    def __init__(self):
        super().__init__('gazebo_robot_control_node')
        cv2.ocl.setUseOpenCL(True)
        self.bridge = CvBridge()

        self.save_images = False
        self.save_base_dir = "/home/aiden/桌面/hook_images_data"
        self.save_interval_sec = 0.3
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

        self.yolo_model = YOLO('/home/aiden/project/research_group/train_robot/runs/obb/custom_obb_train6/weights/best.pt')

        self.update_robot_state = 1
        self.update_picture = 1
        self.update_lidar = 1
        self.counter_robot_state = 0
        self.counter_picture = 0
        self.counter_lidar = 0

        self.start_time = None
        self.current_time = None
        self.car_speed = None
        self.arm_velocity = None
        self.arm_position = None
        self.arm_torque = None

        self.picture_rgb_down = None
        self.picture_dp_down = None
        self.picture_rgb_up = None
        self.picture_dp_up = None
        self.picture_rgb_hand = None
        self.picture_dp_hand = None

        self.depth_down = None
        self.depth_up = None
        self.depth_hand = None

        self.lidar_down = None
        self.lidar_up = None

        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )

        self.robot_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.robot_state_sub_callback,
            10
        )

        picture_topics = ['camera_down_rgb',
                          'camera_down_dp',
                          'camera_hand_rgb',
                          'camera_hand_dp',
                          'camera_up_rgb',
                          'camera_up_dp',
                          ]
        self.pictures_sub = []
        for i in range(len(picture_topics)):
            self.pictures_sub.append(
                self.create_subscription(
                    Image,
                    f'{picture_topics[i]}',
                    lambda msg, idx=i: self.pictures_callback(msg, idx),
                    10
                )
            )

        sub_lidar = False
        if sub_lidar:
            lidar_topics = ['lidar_down', 'lidar_up']
            self.lidar_sub = []
            for i in range(len(lidar_topics)):
                self.lidar_sub.append(
                    self.create_subscription(
                        PointCloud2,
                        f'{lidar_topics[i]}',
                        lambda msg, idx=i: self.lidar_callback(msg, idx),
                        10
                    )
                )

        self.arm_trajectory_pub = self.create_publisher(JointTrajectory, '/velocity_controller/joint_trajectory', 10)
        self.wheel_velocity_pub = self.create_publisher(JointTrajectory, '/base_controller/joint_trajectory', 10)

        self.timer = self.create_timer(0.1, self.control_cycle)
        self.image_show = self.create_timer(0.2, self.visualize_images)
    
    def clock_callback(self, msg):
        if msg is None:
            return
        self.current_time = msg.clock
        if not self.start_time:
            self.start_time = self.current_time
        return
    
    def robot_state_sub_callback(self, msg):
        if msg is None:
            return
        if self.counter_robot_state != 0:
            self.counter_robot_state += 1
            self.counter_robot_state = self.counter_robot_state % self.update_robot_state
            return
        self.counter_robot_state += 1
        self.counter_robot_state = self.counter_robot_state % self.update_robot_state
        
        joints = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'wl1_joint', 'wr1_joint', 'wl2_joint', 'wr2_joint']
        index = []
        for joint in joints:
            index.append(msg.name.index(joint))
        self.car_speed = [msg.velocity[i] for i in index[-4:]]
        self.arm_velocity = [msg.velocity[i] for i in index[:6]]
        self.arm_position = [msg.position[i] for i in index[:6]]
        self.arm_torque = [msg.effort[i] for i in index[:6]]
        return
    
    def pictures_callback(self, msg, idx):
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
    
    def set_car_speed(self, angle_speed):
        msg = JointTrajectory()
        msg.joint_names = ['wl1_joint', 'wr1_joint', 'wl2_joint', 'wr2_joint']
        point = JointTrajectoryPoint()
        point.velocities = [float(v) for v in angle_speed]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000
        msg.points.append(point)
        self.wheel_velocity_pub.publish(msg)
        return
    
    def set_arm_speed(self, angle_speed):
        msg = JointTrajectory()
        msg.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        point = JointTrajectoryPoint()
        point.velocities = [float(v) for v in angle_speed]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000
        msg.points.append(point)
        self.arm_trajectory_pub.publish(msg)
        return
    
    def set_arm_position(self, angle_position):
        msg = JointTrajectory()
        msg.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in angle_position]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        msg.points.append(point)
        self.arm_trajectory_pub.publish(msg)
        return
    
    def control_cycle(self):
        car_speed = 0.0
        self.set_car_speed([car_speed, car_speed, car_speed, car_speed])
        return

    def info_print(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = GazeboRobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()