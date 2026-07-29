#!/usr/bin/env python3
"""
ROS2 节点：读取 waypoints.txt，在 RViz 中可视化 waypoint 标记，
并向 Nav2 PlannerServer 发送 ComputePathToPose 请求。

功能：
  1. 以 MarkerArray 发布 waypoint 球体 + 标签 + 连线
  2. 以 nav_msgs/Path 发布到 rtk_waypoints（供 RTK Spline Planner 构建样条）
  3. 等待 PlannerServer 就绪后，自动发送从第一个到最后一个 waypoint 的规划请求
"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.action import ActionClient

from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from nav2_msgs.action import ComputePathToPose


class WaypointVisualizer(Node):

    def __init__(self):
        super().__init__('waypoint_visualizer')

        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('plan_delay_sec', 6.0)

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
        self.pub_timer = self.create_timer(3.0, self._publish_all)

        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.plan_sent = False
        delay = self.get_parameter('plan_delay_sec').value
        self.plan_timer = self.create_timer(delay, self._try_send_plan)

    # ------------------------------------------------------------------
    # Waypoint I/O
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
    # Publishing
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
            # 球体标记
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

            # 文字标签
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

            # 箭头（航向）
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
            a.scale.x = 3.0  # length
            a.scale.y = 0.4   # width
            a.scale.z = 0.4
            a.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.7)
            ma.markers.append(a)

        # 连线
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
            from geometry_msgs.msg import Point
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
    # Plan request
    # ------------------------------------------------------------------
    def _try_send_plan(self):
        if self.plan_sent or not self.waypoints:
            return

        if not self.plan_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for PlannerServer action...')
            return

        self.plan_sent = True
        self.plan_timer.cancel()

        goal = ComputePathToPose.Goal()
        goal.use_start = True

        goal.start.header.frame_id = 'map'
        goal.start.header.stamp = self.get_clock().now().to_msg()
        goal.start.pose.position.x = self.waypoints[0][0]
        goal.start.pose.position.y = self.waypoints[0][1]

        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = self.waypoints[-1][0]
        goal.goal.pose.position.y = self.waypoints[-1][1]

        goal.planner_id = 'RtkSplinePlanner'

        self.get_logger().info(
            f'Sending plan: ({self.waypoints[0][0]:.1f}, {self.waypoints[0][1]:.1f}) -> '
            f'({self.waypoints[-1][0]:.1f}, {self.waypoints[-1][1]:.1f})')

        future = self.plan_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Plan request rejected!')
            self.plan_sent = False
            return

        self.get_logger().info('Plan request accepted, waiting for result...')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        n = len(result.path.poses)
        t = result.planning_time
        self.get_logger().info(
            f'Plan received: {n} poses, planning time: '
            f'{t.sec}.{t.nanosec // 1_000_000:03d}s')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
