from launch_ros.actions import Node

from launch import LaunchDescription


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
                        "icp_method": "trimmed",
                        "odom_frame": "odom",
                        "use_odom_guess": True,
                        "show_debug_scans": True,
                        "publish_tf": True,
                        "rebase_translation_min_m": 0.05,
                        "rebase_angle_min_deg": 2.0,
                        "rebase_time_min_ms": 1000,
                    }
                ],
                remappings=[("odom_guess", "/odometry/filtered")],
                arguments=["--ros-args", "--log-level", "debug"],
            ),
        ]
    )
