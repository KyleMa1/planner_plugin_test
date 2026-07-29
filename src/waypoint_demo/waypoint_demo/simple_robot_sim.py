#!/usr/bin/env python3
"""
轻量级机器人运动学仿真节点。

功能：
  - 订阅 /cmd_vel，按差速/履带运动学积分位姿
  - 发布 /odom (nav_msgs/Odometry)
  - 广播 TF: odom -> base_link
  - 无需 Gazebo，纯数值仿真
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SimpleRobotSim(Node):

    def __init__(self):
        super().__init__('simple_robot_sim')

        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)
        self.declare_parameter('odom_rate', 50.0)
        self.declare_parameter('max_linear_vel', 3.0)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('linear_velocity_noise_stddev', 0.01)
        self.declare_parameter('angular_velocity_noise_stddev', 0.01)
        self.declare_parameter('noise_seed', 42)

        self.x = self.get_parameter('x0').value
        self.y = self.get_parameter('y0').value
        self.theta = self.get_parameter('theta0').value
        rate = self.get_parameter('odom_rate').value
        self.max_v = self.get_parameter('max_linear_vel').value
        self.max_w = self.get_parameter('max_angular_vel').value
        self.linear_noise_stddev = self.get_parameter(
            'linear_velocity_noise_stddev').value
        self.angular_noise_stddev = self.get_parameter(
            'angular_velocity_noise_stddev').value
        self.noise_generator = random.Random(
            self.get_parameter('noise_seed').value)

        self.vx = 0.0
        self.wz = 0.0

        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_cb, 10)

        odom_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.odom_pub = self.create_publisher(Odometry, 'odom', odom_qos)
        self.robot_marker_pub = self.create_publisher(
            Marker, 'robot_marker', odom_qos)
        self.trajectory_pub = self.create_publisher(
            Path, 'actual_trajectory', odom_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.trajectory = Path()
        self.trajectory.header.frame_id = 'map'
        self.last_trajectory_x = self.x
        self.last_trajectory_y = self.y

        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self._update)
        self.last_time = self.get_clock().now()

        self.get_logger().info(
            f'SimpleRobotSim started at ({self.x:.1f}, {self.y:.1f}, '
            f'{math.degrees(self.theta):.1f}°), rate={rate}Hz')

    def _cmd_vel_cb(self, msg: Twist):
        self.vx = max(-self.max_v, min(self.max_v, msg.linear.x))
        self.wz = max(-self.max_w, min(self.max_w, msg.angular.z))

    def _update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            dt = self.dt

        # 在控制指令上叠加零均值高斯速度噪声，再进行运动学积分
        actual_vx = max(
            -self.max_v,
            min(
                self.max_v,
                self.vx + self.noise_generator.gauss(
                    0.0, self.linear_noise_stddev)))
        actual_wz = max(
            -self.max_w,
            min(
                self.max_w,
                self.wz + self.noise_generator.gauss(
                    0.0, self.angular_noise_stddev)))
        self.theta += actual_wz * dt
        self.x += actual_vx * math.cos(self.theta) * dt
        self.y += actual_vx * math.sin(self.theta) * dt

        stamp = now.to_msg()
        q = yaw_to_quaternion(self.theta)

        # 发布 TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # 发布 Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = actual_vx
        odom.twist.twist.angular.z = actual_wz
        self.odom_pub.publish(odom)

        # 发布醒目的机器人箭头，便于在大范围地图中观察跟踪运动
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'map'
        marker.ns = 'sim_robot'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.8
        marker.pose.orientation = q
        marker.scale.x = 6.0
        marker.scale.y = 2.5
        marker.scale.z = 1.5
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.robot_marker_pub.publish(marker)

        # 每移动 0.25 m 记录一个点，发布实际行驶轨迹
        moved = math.hypot(
            self.x - self.last_trajectory_x,
            self.y - self.last_trajectory_y)
        if not self.trajectory.poses or moved >= 0.25:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.x
            pose.pose.position.y = self.y
            pose.pose.orientation = q
            self.trajectory.poses.append(pose)
            self.trajectory.header.stamp = stamp
            self.trajectory_pub.publish(self.trajectory)
            self.last_trajectory_x = self.x
            self.last_trajectory_y = self.y


def main(args=None):
    rclpy.init(args=args)
    node = SimpleRobotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
