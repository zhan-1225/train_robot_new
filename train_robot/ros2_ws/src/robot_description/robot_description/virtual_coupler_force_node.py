import math
import os
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def matmul(a, b):
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(4))
    return out


def transform_point(t, p):
    return [
        t[0][0] * p[0] + t[0][1] * p[1] + t[0][2] * p[2] + t[0][3],
        t[1][0] * p[0] + t[1][1] * p[1] + t[1][2] * p[2] + t[1][3],
        t[2][0] * p[0] + t[2][1] * p[1] + t[2][2] * p[2] + t[2][3],
    ]


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def origin_to_transform(origin):
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if origin is not None:
        xyz = [float(v) for v in origin.attrib.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.attrib.get('rpy', '0 0 0').split()]
    rot = rpy_to_matrix(rpy[0], rpy[1], rpy[2])
    return [
        [rot[0][0], rot[0][1], rot[0][2], xyz[0]],
        [rot[1][0], rot[1][1], rot[1][2], xyz[1]],
        [rot[2][0], rot[2][1], rot[2][2], xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def axis_angle_transform(axis, angle):
    norm = math.sqrt(sum(v * v for v in axis))
    if norm < 1e-9:
        return identity()
    x, y, z = [v / norm for v in axis]
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return [
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s, 0.0],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s, 0.0],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def identity():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def pose_to_transform(x, y, z, yaw):
    t = origin_to_transform(None)
    rot = rpy_to_matrix(0.0, 0.0, yaw)
    for i in range(3):
        for j in range(3):
            t[i][j] = rot[i][j]
    t[0][3] = x
    t[1][3] = y
    t[2][3] = z
    return t


def vec_add(a, b):
    return [a[i] + b[i] for i in range(3)]


def vec_sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def vec_scale(a, s):
    return [a[i] * s for i in range(3)]


def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def norm(a):
    return math.sqrt(dot(a, a))


def normalize(a, fallback):
    n = norm(a)
    if n < 1e-9:
        return fallback[:]
    return [v / n for v in a]


class VirtualCouplerForceNode(Node):
    """Penalty-contact virtual force model for gripper-coupler interaction."""

    def __init__(self):
        super().__init__('virtual_coupler_force_node')

        share_dir = get_package_share_directory('robot_description')
        default_urdf = os.path.join(share_dir, 'urdf', 'robot_complete_gazebo.urdf')

        self.declare_parameter('urdf_path', default_urdf)
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tool_link', 'hooker_link')
        self.declare_parameter('tool_contact_offset', [-0.3305, 0.017, 1.284])
        self.declare_parameter('robot_pose', [2.87, -168.884903, 3.47, 0.0])
        self.declare_parameter('coupler_pose', [3.5, -170.0, 4.735, -1.5708])
        self.declare_parameter('coupler_contact_offset', [0.0, -0.652, 1.304])
        self.declare_parameter('contact_axis_world', [0.0, 1.0, 0.0])
        self.declare_parameter('free_gap', 0.12)
        self.declare_parameter('lateral_window', 0.35)
        self.declare_parameter('k_load', 8000.0)
        self.declare_parameter('k_unload', 4500.0)
        self.declare_parameter('d_load', 180.0)
        self.declare_parameter('d_unload', 90.0)
        self.declare_parameter('mu', 0.28)
        self.declare_parameter('stiction_velocity', 0.02)
        self.declare_parameter('force_limit', 2500.0)
        self.declare_parameter('filter_alpha', 0.25)
        self.declare_parameter('publish_rate', 100.0)

        self.joints = self._load_chain(
            self.get_parameter('urdf_path').value,
            self.get_parameter('base_link').value,
            self.get_parameter('tool_link').value,
        )
        self.joint_positions = {}
        self.last_time = None
        self.last_compression = 0.0
        self.last_tool_pos = None
        self.filtered_force = [0.0, 0.0, 0.0]

        self.force_pub = self.create_publisher(WrenchStamped, 'virtual_coupler_force', 10)
        self.state_pub = self.create_publisher(Float64MultiArray, 'virtual_coupler_contact_state', 10)
        self.create_subscription(JointState, 'joint_states', self._joint_state_cb, 10)

        period = 1.0 / float(self.get_parameter('publish_rate').value)
        self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            'Virtual coupler force node started. Publishing /virtual_coupler_force'
        )

    def _load_chain(self, urdf_path, base_link, tool_link):
        root = ET.parse(urdf_path).getroot()
        parent_map = {}
        for joint in root.findall('joint'):
            child = joint.find('child').attrib['link']
            parent_map[child] = joint

        chain = []
        current = tool_link
        while current != base_link:
            joint = parent_map.get(current)
            if joint is None:
                raise RuntimeError(f'No joint path from {base_link} to {tool_link}')
            parent = joint.find('parent').attrib['link']
            axis_el = joint.find('axis')
            axis = [0.0, 0.0, 1.0]
            if axis_el is not None:
                axis = [float(v) for v in axis_el.attrib.get('xyz', '0 0 1').split()]
            chain.append({
                'name': joint.attrib['name'],
                'type': joint.attrib.get('type', 'fixed'),
                'origin': origin_to_transform(joint.find('origin')),
                'axis': axis,
                'parent': parent,
                'child': current,
            })
            current = parent
        chain.reverse()
        return chain

    def _joint_state_cb(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position

    def _tool_transform_base(self):
        t = identity()
        for joint in self.joints:
            t = matmul(t, joint['origin'])
            if joint['type'] in ('revolute', 'continuous'):
                angle = self.joint_positions.get(joint['name'], 0.0)
                t = matmul(t, axis_angle_transform(joint['axis'], angle))
        return t

    def _timer_cb(self):
        now = self.get_clock().now()
        dt = 0.0
        if self.last_time is not None:
            dt = max((now - self.last_time).nanoseconds * 1e-9, 1e-6)
        self.last_time = now

        robot_pose = [float(v) for v in self.get_parameter('robot_pose').value]
        coupler_pose = [float(v) for v in self.get_parameter('coupler_pose').value]
        robot_world = pose_to_transform(*robot_pose)
        coupler_world = pose_to_transform(*coupler_pose)

        tool_offset = [float(v) for v in self.get_parameter('tool_contact_offset').value]
        coupler_offset = [float(v) for v in self.get_parameter('coupler_contact_offset').value]
        tool_world = matmul(robot_world, self._tool_transform_base())
        tool_pos = transform_point(tool_world, tool_offset)
        coupler_pos = transform_point(coupler_world, coupler_offset)

        axis = normalize(
            [float(v) for v in self.get_parameter('contact_axis_world').value],
            [0.0, 1.0, 0.0],
        )
        rel = vec_sub(tool_pos, coupler_pos)
        normal_distance = dot(rel, axis)
        lateral = vec_sub(rel, vec_scale(axis, normal_distance))
        lateral_distance = norm(lateral)
        lateral_window = float(self.get_parameter('lateral_window').value)
        free_gap = float(self.get_parameter('free_gap').value)

        compression = max(0.0, free_gap - normal_distance)
        if lateral_distance > lateral_window:
            compression = 0.0

        compression_rate = 0.0
        tool_velocity = [0.0, 0.0, 0.0]
        if dt > 0.0:
            compression_rate = (compression - self.last_compression) / dt
            if self.last_tool_pos is not None:
                tool_velocity = vec_scale(vec_sub(tool_pos, self.last_tool_pos), 1.0 / dt)
        self.last_compression = compression
        self.last_tool_pos = tool_pos

        if compression_rate >= 0.0:
            stiffness = float(self.get_parameter('k_load').value)
            damping = float(self.get_parameter('d_load').value)
        else:
            stiffness = float(self.get_parameter('k_unload').value)
            damping = float(self.get_parameter('d_unload').value)

        normal_force = 0.0
        if compression > 0.0:
            normal_force = stiffness * compression + damping * compression_rate
            normal_force = max(0.0, normal_force)
            normal_force = min(normal_force, float(self.get_parameter('force_limit').value))

        tangent_dir = normalize(lateral, [1.0, 0.0, 0.0])
        tangent_speed = dot(tool_velocity, tangent_dir)
        mu = float(self.get_parameter('mu').value)
        stiction_velocity = max(float(self.get_parameter('stiction_velocity').value), 1e-6)
        friction_mag = -mu * normal_force * math.tanh(tangent_speed / stiction_velocity)

        force = vec_add(vec_scale(axis, normal_force), vec_scale(tangent_dir, friction_mag))
        alpha = float(self.get_parameter('filter_alpha').value)
        alpha = min(max(alpha, 0.0), 1.0)
        self.filtered_force = [
            alpha * force[i] + (1.0 - alpha) * self.filtered_force[i] for i in range(3)
        ]

        wrench = WrenchStamped()
        wrench.header.stamp = now.to_msg()
        wrench.header.frame_id = 'world'
        wrench.wrench.force.x = self.filtered_force[0]
        wrench.wrench.force.y = self.filtered_force[1]
        wrench.wrench.force.z = self.filtered_force[2]
        self.force_pub.publish(wrench)

        state = Float64MultiArray()
        state.data = [
            compression,
            compression_rate,
            normal_force,
            lateral_distance,
            normal_distance,
            1.0 if compression > 0.0 else 0.0,
            self.filtered_force[0],
            self.filtered_force[1],
            self.filtered_force[2],
        ]
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualCouplerForceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
