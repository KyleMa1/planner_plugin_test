"""
Pure Pursuit 导航 Demo Launch 文件

启动内容：
  1. simple_robot_sim   — 轻量运动学仿真（odom + TF: odom->base_link）
  2. static TF          — map -> odom 恒等变换
  3. planner_server     — RTK Spline Planner
  4. controller_server  — Regulated Pure Pursuit Controller
  5. bt_navigator       — BT 导航器（使用自定义简易行为树）
  6. lifecycle_manager  — 自动 bring-up 以上三个 Nav2 节点
  7. nav_demo           — 发布 waypoints + 发送 NavigateToPose
  8. rviz2              — 可视化

用法：
  ros2 launch nav2_rtk_spline_planner pure_pursuit_demo.launch.py
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _inject_bt_paths(yaml_path, bt_xml, bt_through_poses_xml):
    """读取原始 YAML，注入 BT XML 路径，返回临时文件路径。"""
    with open(yaml_path, 'r') as f:
        params = yaml.safe_load(f)

    bt_section = params.setdefault('bt_navigator', {}).setdefault('ros__parameters', {})
    # Nav2 Humble 使用 default_nav_to_pose_bt_xml；旧的
    # default_bt_xml_filename 不会被 NavigateToPoseNavigator 读取。
    bt_section['default_nav_to_pose_bt_xml'] = bt_xml
    bt_section['default_nav_through_poses_bt_xml'] = bt_through_poses_xml

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='nav2_pursuit_', delete=False)
    yaml.dump(params, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


def generate_launch_description():
    planner_pkg = get_package_share_directory('nav2_rtk_spline_planner')
    demo_pkg = get_package_share_directory('waypoint_demo')

    nav2_params_file = os.path.join(planner_pkg, 'config', 'nav2_pursuit_params.yaml')
    bt_xml_file = os.path.join(planner_pkg, 'config', 'navigate_spline.xml')
    bt_through_poses_xml = os.path.join(
        planner_pkg, 'config', 'navigate_through_poses_simple.xml')
    rviz_config_file = os.path.join(planner_pkg, 'config', 'pursuit_demo.rviz')
    default_waypoint_file = os.path.join(demo_pkg, 'data', 'waypoints.txt')

    patched_params = _inject_bt_paths(nav2_params_file, bt_xml_file, bt_through_poses_xml)

    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=default_waypoint_file,
        description='Path to waypoints txt file (x y theta per line)',
    )

    robot_sim = Node(
        package='waypoint_demo',
        executable='simple_robot_sim',
        name='simple_robot_sim',
        output='screen',
        parameters=[{
            'x0': 0.0,
            'y0': 0.0,
            'theta0': 0.0,
            'odom_rate': 50.0,
            'linear_velocity_noise_stddev': 0.01,
            'angular_velocity_noise_stddev': 0.01,
            'noise_seed': 42,
        }],
    )

    # obstacle_publisher = Node(
    #     package='waypoint_demo',
    #     executable='obstacle_publisher',
    #     name='obstacle_publisher',
    #     output='screen',
    #     parameters=[{
    #         'obstacle_x': [35.0, 104.0, 180.0],
    #         'obstacle_y': [0.0, 50.0, 100.0],
    #         'obstacle_radius': 2.0,
    #     }],
    # )

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
        output='screen',
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[patched_params],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[patched_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[patched_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[patched_params],
    )

    nav_demo = Node(
        package='waypoint_demo',
        executable='nav_demo',
        name='nav_demo',
        output='screen',
        parameters=[{
            'waypoint_file': LaunchConfiguration('waypoint_file'),
            'nav_delay_sec': 15.0,
        }],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    return LaunchDescription([
        waypoint_file_arg,
        static_tf_map_odom,
        robot_sim,
        # obstacle_publisher,
        planner_server,
        controller_server,
        bt_navigator,
        lifecycle_manager,
        nav_demo,
        rviz2,
    ])
