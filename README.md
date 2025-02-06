# lidar_odometry

Package to produce odometry from (2D) LiDAR data. Uses [libcevicp](https://github.com/cornellev/icp).

## Building
You must install the CEV ICP library from commit [a02c13c](https://github.com/cornellev/icp/tree/a02c13c6020af7405ed7c820112746184f381a0a). You'll probably have the best time if you use the default install directory. There are ROS 2 dependencies as well, but `rosdep` should help you with those.

## Parameters
* `odom_frame` - The name of the odometry frame to publish odom info with.
* `rebase_translation_min_cm` - The minimum translation required between scans to trigger an odometry update. 
* `rebase_angle_min_rad` - The minimum rotation angle required between scans to trigger an odometry update.
> [!WARNING]  
> Setting these too low may lead to small and incorrect scan deltas that, when integrated, don't produce the correct result. Setting these too high may prevent scans from matching all together.
* `rebase_time_min_ms` - The minimum time required between scans to trigger an odometry update.
> [!NOTE]
> *Any* of the `rebase` thresholds being passed will trigger an odometry update.
* `publish_tf` - Publish TF data in addition to odometry messages.
* `show_debug_scans` - Publish base, transformed, and current scans to debug scan matching performance.
