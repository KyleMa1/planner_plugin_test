#!/usr/bin/env python3
"""发布同时用于 Nav2 costmap 和 RViz 的静态演示障碍物。"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


class ObstaclePublisher(Node):

    def __init__(self):
        super().__init__('obstacle_publisher')

        self.declare_parameter('scenario', 'circles')
        self.declare_parameter('obstacle_x', [35.0, 104.0, 180.0])
        self.declare_parameter('obstacle_y', [0.0, 50.0, 100.0])
        self.declare_parameter('obstacle_radius', 2.0)
        self.declare_parameter('barrier_x', 35.0)
        self.declare_parameter('gap_width', 1.6)
        self.declare_parameter('center_block_y_bias', 0.0)

        self.scenario = self.get_parameter('scenario').value
        xs = list(self.get_parameter('obstacle_x').value)
        ys = list(self.get_parameter('obstacle_y').value)
        self.radius = float(self.get_parameter('obstacle_radius').value)
        if len(xs) != len(ys):
            raise ValueError('obstacle_x and obstacle_y must have equal lengths')
        self.obstacles = list(zip(xs, ys))
        self.shapes = self._build_scenario()

        cloud_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        marker_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, 'demo_obstacles', cloud_qos)
        self.marker_pub = self.create_publisher(
            MarkerArray, 'obstacle_markers', marker_qos)

        self.points = self._build_point_cloud()
        self.timer = self.create_timer(0.5, self._publish)
        self._publish()
        self.get_logger().info(
            f'Publishing obstacle scenario "{self.scenario}" with '
            f'{len(self.shapes)} shapes on /demo_obstacles')

    def _build_scenario(self):
        if self.scenario != 'dual_gap':
            return [
                {
                    'type': 'circle',
                    'x': x,
                    'y': y,
                    'radius': self.radius,
                }
                for x, y in self.obstacles
            ]

        # 中央阻挡块和上下边界形成两条对称窄通道。
        # 中央块外缘为 y=±1.5；边界内缘为 ±(1.5 + gap_width)。
        barrier_x = float(self.get_parameter('barrier_x').value)
        gap_width = float(self.get_parameter('gap_width').value)
        center_y = float(
            self.get_parameter('center_block_y_bias').value)
        center_half_height = 1.5
        wall_thickness = 2.0
        wall_center_y = (
            center_half_height + gap_width + wall_thickness / 2.0)
        return [
            {
                'type': 'box',
                'x': barrier_x,
                'y': center_y,
                'size_x': 4.0,
                'size_y': 2.0 * center_half_height,
            },
            {
                'type': 'box',
                'x': barrier_x,
                'y': wall_center_y,
                'size_x': 14.0,
                'size_y': wall_thickness,
            },
            {
                'type': 'box',
                'x': barrier_x,
                'y': -wall_center_y,
                'size_x': 14.0,
                'size_y': wall_thickness,
            },
        ]

    def _build_point_cloud(self):
        """用密集圆盘点云表示障碍，确保 costmap 内部也被标记。"""
        points = []
        spacing = 0.25
        for shape in self.shapes:
            if shape['type'] == 'circle':
                radius = shape['radius']
                steps = int(math.ceil(radius / spacing))
                for ix in range(-steps, steps + 1):
                    for iy in range(-steps, steps + 1):
                        dx = ix * spacing
                        dy = iy * spacing
                        if dx * dx + dy * dy <= radius * radius:
                            points.append((
                                shape['x'] + dx, shape['y'] + dy, 0.2))
            else:
                x_steps = int(math.ceil(shape['size_x'] / spacing))
                y_steps = int(math.ceil(shape['size_y'] / spacing))
                x_min = shape['x'] - shape['size_x'] / 2.0
                y_min = shape['y'] - shape['size_y'] / 2.0
                for ix in range(x_steps + 1):
                    for iy in range(y_steps + 1):
                        points.append((
                            x_min + ix * spacing,
                            y_min + iy * spacing,
                            0.2,
                        ))
        return points

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id='map')
        self.cloud_pub.publish(
            point_cloud2.create_cloud_xyz32(header, self.points))

        markers = MarkerArray()
        for index, shape in enumerate(self.shapes):
            marker = Marker()
            marker.header = header
            marker.ns = 'demo_obstacles'
            marker.id = index
            marker.type = (
                Marker.CYLINDER
                if shape['type'] == 'circle' else Marker.CUBE)
            marker.action = Marker.ADD
            marker.pose.position.x = shape['x']
            marker.pose.position.y = shape['y']
            marker.pose.position.z = 1.0
            marker.pose.orientation.w = 1.0
            if shape['type'] == 'circle':
                marker.scale.x = 2.0 * shape['radius']
                marker.scale.y = 2.0 * shape['radius']
            else:
                marker.scale.x = shape['size_x']
                marker.scale.y = shape['size_y']
            marker.scale.z = 2.0
            marker.color.r = 0.9
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 0.9
            markers.markers.append(marker)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
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
