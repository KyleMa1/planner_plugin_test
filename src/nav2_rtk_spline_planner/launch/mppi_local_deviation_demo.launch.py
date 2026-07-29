"""RTK Spline + MPPI 双侧窄通道局部偏离演示。"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _inject_bt_paths(yaml_path, bt_xml, bt_through_poses_xml):
    with open(yaml_path, 'r') as stream:
        params = yaml.safe_load(stream)

    bt_params = params.setdefault(
        'bt_navigator', {}).setdefault('ros__parameters', {})
    bt_params['default_nav_to_pose_bt_xml'] = bt_xml
    bt_params['default_nav_through_poses_bt_xml'] = bt_through_poses_xml

    temporary = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='nav2_mppi_', delete=False)
    yaml.dump(params, temporary, default_flow_style=False)
    temporary.close()
    return temporary.name


def generate_launch_description():
    planner_pkg = get_package_share_directory('nav2_rtk_spline_planner')
    demo_pkg = get_package_share_directory('waypoint_demo')

    params_source = os.path.join(
        planner_pkg, 'config', 'nav2_mppi_params.yaml')
    navigate_bt = os.path.join(
        planner_pkg, 'config', 'navigate_spline_slow_replan.xml')
    through_poses_bt = os.path.join(
        planner_pkg, 'config', 'navigate_through_poses_simple.xml')
    rviz_config = os.path.join(
        planner_pkg, 'config', 'mppi_deviation_demo.rviz')
    default_waypoints = os.path.join(
        demo_pkg, 'data', 'waypoints.txt')
    configured_params = _inject_bt_paths(
        params_source, navigate_bt, through_poses_bt)

    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=default_waypoints,
        description='Path to waypoints txt file (x y theta per line)',
    )
    gap_width_arg = DeclareLaunchArgument(
        'gap_width',
        # robot_radius(0.5)*2 + inflation(0.4)*2 ≈ 1.8 m 为下限；
        # 默认 2.2 m 留出余量，避免 MPPI 在入口原地打转。
        default_value='2.2',
        description='Physical width of each local bypass corridor in meters',
    )
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='-1.0',
        description='Optional demo goal X; negative X/Y uses final waypoint',
    )
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='-1.0',
        description='Optional demo goal Y; negative X/Y uses final waypoint',
    )
    start_x_arg = DeclareLaunchArgument(
        'start_x',
        default_value='0.0',
        description='Initial simulated robot X position',
    )

    map_to_odom = Node(
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

    robot_sim = Node(
        package='waypoint_demo',
        executable='simple_robot_sim',
        name='simple_robot_sim',
        output='screen',
        parameters=[{
            'x0': ParameterValue(
                LaunchConfiguration('start_x'), value_type=float),
            'y0': 0.0,
            'theta0': 0.0,
            'odom_rate': 50.0,
            'linear_velocity_noise_stddev': 0.0,
            'angular_velocity_noise_stddev': 0.0,
            'noise_seed': 42,
        }],
    )

    obstacles = Node(
        package='waypoint_demo',
        executable='obstacle_publisher',
        name='obstacle_publisher',
        output='screen',
        parameters=[{
            'scenario': 'dual_gap',
            'barrier_x': 35.0,
            'gap_width': ParameterValue(
                LaunchConfiguration('gap_width'), value_type=float),
            # 两侧仍可通行，用 0.5 m 几何差异稳定选择较宽通道，
            # 避免 MPPI 在完全对称的两个局部最优解之间来回切换。
            'center_block_y_bias': 0.5,
        }],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[configured_params],
    )

    nav_demo = Node(
        package='waypoint_demo',
        executable='nav_demo',
        name='nav_demo',
        output='screen',
        parameters=[{
            'waypoint_file': LaunchConfiguration('waypoint_file'),
            'nav_delay_sec': 15.0,
            'goal_x': ParameterValue(
                LaunchConfiguration('goal_x'), value_type=float),
            'goal_y': ParameterValue(
                LaunchConfiguration('goal_y'), value_type=float),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        waypoint_file_arg,
        gap_width_arg,
        goal_x_arg,
        goal_y_arg,
        start_x_arg,
        map_to_odom,
        robot_sim,
        obstacles,
        planner_server,
        controller_server,
        bt_navigator,
        lifecycle_manager,
        nav_demo,
        rviz,
    ])
