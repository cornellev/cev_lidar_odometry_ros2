import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Locate the config file
    pkg_share = FindPackageShare(package='lidar_odometry').find('lidar_odometry')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        # 1. Custom Motor Controller 
        Node(
            package="lidar_odometry",
            executable="motor_controller",
            name="motor_controller_node",
            output="screen",
        ),
        
        # 2. Official RPLIDAR Driver 
        Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            output="screen",
            parameters=[{
                "serial_port": "/dev/ttyAMA0",  
                "serial_baudrate": 115200,      
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard"
            }],
        ),

        # 3. WitMotion IMU Driver
        Node(
            package='witmotion_ros2',
            executable='witmotion_ros2',
            name='witmotion_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud_rate': 115200, 
                'rate': 50.0
            }]
        ),

        # 4A. The IMU Covariance Injector (Hacks the 0.0 values)
        Node(
            package='lidar_odometry',
            executable='imu_covariance_injector.py',
            name='imu_covariance_injector',
            output='screen'
        ),

        # 4B. The UKF Localization Node (Upgraded from EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',           # <-- Change this to ekf_node
            name='ekf_filter_node',          # <-- Change this to ekf_filter_node
            output='screen',
            parameters=[ekf_config_path],
            remappings=[('/imu/data', '/imu/data_with_covariance')]
        ),
        # 5. Static Transforms
        # base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),
        # base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # 6. Your ICP Odometry Node (Baton passed to UKF)
        Node(
            package="lidar_odometry",
            executable="lidar_odometry",
            name="lidar_odometry_node",
            output="screen",
            parameters=[
                {
                    "icp_method": "trimmed",
                    "odom_frame": "odom",
                    "use_odom_guess": False,
                    "show_debug_scans": True,
                    "publish_tf": False,     # CRITICAL: UKF is back in control of TF
                    "rebase_translation_min_m": 0.05,
                    "rebase_angle_min_deg": 2.0,
                    "rebase_time_min_ms": 50,
                }
            ],
            remappings=[("odom_guess", "/odometry/filtered")],
        ),

        # 7. The Foxglove WebSocket Bridge
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                "port": 8765,
                "address": "0.0.0.0",
            }],
        ),

        # 8. PGO Backend
        Node(
            package="cev_pose_graph_slam",
            executable="pgo_node",
            name="pgo_backend",
            output="screen",
            # We hardcoded C++ to listen to /odom, so we remap it here 
            # to listen to the UKF output instead!
            remappings=[("/odom", "/odometry/filtered")] 
        )
    ])