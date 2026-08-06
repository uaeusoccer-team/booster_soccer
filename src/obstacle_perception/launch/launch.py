import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("obstacle_perception")
    config = os.path.join(package_share, "config", "obstacle_perception.yaml")
    return LaunchDescription(
        [
            Node(
                package="obstacle_perception",
                executable="depth_obstacle_node",
                name="depth_obstacle_node",
                output="screen",
                parameters=[config],
            )
        ]
    )
