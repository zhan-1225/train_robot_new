import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class DefaultJointStatePublisher(Node):
    def __init__(self):
        super().__init__('default_joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        default_joint_names = [
            'j1',
            'j2',
            'j3',
            'j4',
            'j5',
            'j6',
            'wl1_joint',
            'wl2_joint',
            'wr1_joint',
            'wr2_joint',
        ]
        self.declare_parameter('joint_names', default_joint_names)
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.timer = self.create_timer(0.05, self.publish_joint_states)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DefaultJointStatePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
