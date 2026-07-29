from setuptools import setup
import os
from glob import glob

package_name = 'waypoint_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='developer@example.com',
    description='Waypoint generation and visualization for RTK Spline Planner',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'generate_waypoints = waypoint_demo.generate_waypoints:main',
            'waypoint_visualizer = waypoint_demo.waypoint_visualizer:main',
            'simple_robot_sim = waypoint_demo.simple_robot_sim:main',
            'nav_demo = waypoint_demo.nav_demo:main',
            'obstacle_publisher = waypoint_demo.obstacle_publisher:main',
        ],
    },
)
