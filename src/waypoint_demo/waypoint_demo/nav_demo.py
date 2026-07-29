#!/usr/bin/env python3
"""
Pure Pursuit 导航 Demo 节点。

功能：
  1. 读取 waypoints.txt，发布 MarkerArray + nav_msgs/Path 到 rtk_waypoints
  2. 等待 Nav2 就绪，发送 NavigateToPose 到 bt_navigator
  3. 实时反馈导航进度
"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.action import ActionClient

from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA
from nav2_msgs.action import NavigateToPose


class NavDemo(Node):

    def __init__(self):
        super().__init__('nav_demo')

        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('nav_delay_sec', 10.0)
        self.declare_parameter('goal_x', -1.0)
        self.declare_parameter('goal_y', -1.0)

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', latched_qos)
        self.path_pub = self.create_publisher(Path, 'rtk_waypoints', latched_qos)

        self.waypoints = self._load_waypoints()
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded — aborting')
            return

        self._publish_all()
        self.pub_timer = self.create_timer(5.0, self._publish_all)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_sent = False
        delay = self.get_parameter('nav_delay_sec').value
        self.nav_timer = self.create_timer(delay, self._try_navigate)

    # ------------------------------------------------------------------
    def _load_waypoints(self):
        filepath = self.get_parameter('waypoint_file').value
        if not filepath or not os.path.isfile(filepath):
            self.get_logger().error(f'Waypoint file not found: "{filepath}"')
            return []

        waypoints = []
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) >= 2:
                    x, y = float(parts[0]), float(parts[1])
                    theta = float(parts[2]) if len(parts) >= 3 else 0.0
                    waypoints.append((x, y, theta))

        self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {filepath}')
        return waypoints

    # ------------------------------------------------------------------
    def _publish_all(self):
        if not self.waypoints:
            return
        self._publish_markers()
        self._publish_path()

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()

        for i, (x, y, theta) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.3
            m.scale.x = m.scale.y = m.scale.z = 2.0
            m.color = ColorRGBA(r=1.0, g=0.3, b=0.1, a=0.85)
            ma.markers.append(m)

            t = Marker()
            t.header = m.header
            t.ns = 'labels'
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = x
            t.pose.position.y = y
            t.pose.position.z = 3.5
            t.scale.z = 1.5
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = f'WP{i}'
            ma.markers.append(t)

            a = Marker()
            a.header = m.header
            a.ns = 'headings'
            a.id = i
            a.type = Marker.ARROW
            a.action = Marker.ADD
            a.pose.position.x = x
            a.pose.position.y = y
            a.pose.position.z = 0.2
            a.pose.orientation.z = math.sin(theta / 2.0)
            a.pose.orientation.w = math.cos(theta / 2.0)
            a.scale.x = 3.0
            a.scale.y = 0.4
            a.scale.z = 0.4
            a.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.7)
            ma.markers.append(a)

        line = Marker()
        line.header.frame_id = 'map'
        line.header.stamp = now
        line.ns = 'connections'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.3
        line.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.5)
        for x, y, _ in self.waypoints:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.1
            line.points.append(p)
        ma.markers.append(line)

        self.marker_pub.publish(ma)

    def _publish_path(self):
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = now

        for x, y, theta in self.waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(theta / 2.0)
            ps.pose.orientation.w = math.cos(theta / 2.0)
            path.poses.append(ps)

        self.path_pub.publish(path)

    # ------------------------------------------------------------------
    def _try_navigate(self):
        if self.nav_sent or not self.waypoints:
            return

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for navigate_to_pose action server...')
            return

        self.nav_sent = True
        self.nav_timer.cancel()

        gx = self.get_parameter('goal_x').value
        gy = self.get_parameter('goal_y').value
        if gx < 0 and gy < 0:
            gx = self.waypoints[-1][0]
            gy = self.waypoints[-1][1]
            goal_theta = self.waypoints[-1][2]
        else:
            goal_theta = 0.0

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(goal_theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(goal_theta / 2.0)

        self.get_logger().info(
            f'Sending NavigateToPose goal: ({gx:.1f}, {gy:.1f}, theta={goal_theta:.3f})')
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            self.nav_sent = False
            return

        self.get_logger().info('Navigation goal accepted')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self.get_logger().info(f'Distance remaining: {dist:.1f} m', throttle_duration_sec=3.0)

    def _result_cb(self, future):
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation SUCCEEDED!')
        elif status == 5:  # CANCELED
            self.get_logger().warn('Navigation CANCELED')
        else:
            self.get_logger().error(f'Navigation FAILED (status={status})')


def main(args=None):
    rclpy.init(args=args)
    node = NavDemo()
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
