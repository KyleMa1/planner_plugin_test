"""
RTK Spline Planner 可视化 Demo Launch 文件

启动内容：
  1. static_transform_publisher  — 发布 map -> base_link 静态 TF
  2. planner_server              — Nav2 PlannerServer + RTK Spline Planner Plugin
  3. lifecycle_manager           — 自动 bring-up planner_server
  4. waypoint_visualizer         — 读取 waypoints.txt、发布标记、发送规划请求
  5. rviz2                       — 可视化

用法：
  ros2 launch nav2_rtk_spline_planner rtk_spline_demo.launch.py

  可选参数：
  ros2 launch nav2_rtk_spline_planner rtk_spline_demo.launch.py \
      waypoint_file:=/path/to/custom/waypoints.txt
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planner_pkg = get_package_share_directory('nav2_rtk_spline_planner')
    demo_pkg = get_package_share_directory('waypoint_demo')

    nav2_params_file = os.path.join(planner_pkg, 'config', 'nav2_demo_params.yaml')
    rviz_config_file = os.path.join(planner_pkg, 'config', 'demo.rviz')
    default_waypoint_file = os.path.join(demo_pkg, 'data', 'waypoints.txt')

    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=default_waypoint_file,
        description='Path to waypoints txt file (x y theta per line)',
    )

    # 1. Static TF: map -> base_link (机器人静止在原点，仅用于 demo)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen',
    )

    # 2. Nav2 Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    # 3. Lifecycle Manager — 自动激活 planner_server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[nav2_params_file],
    )

    # 4. Waypoint 可视化 + 自动规划请求
    waypoint_visualizer = Node(
        package='waypoint_demo',
        executable='waypoint_visualizer',
        name='waypoint_visualizer',
        output='screen',
        parameters=[{
            'waypoint_file': LaunchConfiguration('waypoint_file'),
            'plan_delay_sec': 8.0,
        }],
    )

    # 5. RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    return LaunchDescription([
        waypoint_file_arg,
        static_tf,
        planner_server,
        lifecycle_manager,
        waypoint_visualizer,
        rviz2,
    ])
