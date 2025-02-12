import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def get_path(package, dir, file):
    return os.path.join(get_package_share_directory(package), dir, file)


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lidar_odometry",
                executable="lidar_odometry",
                name="lidar_odometry_node",
                output="screen",
                parameters=[
                    {
                        "config_file": get_path(
                            "lidar_odometry", "config", "example.yaml"
                        )
                    }
                ],
            )
        ]
    )
