# livox_ros_driver2

ROS 2-only driver for Livox HAP, Mid-360, and Mid-360S LiDARs.

## Build

Install Livox-SDK2 first, then build this package from the workspace:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select livox_ros_driver2 --symlink-install
source install/setup.bash
```

The local helper performs the same package build:

```bash
./src/sensor_support/livox_ros_driver2/build.sh
```

## Mid-360

Edit `config/MID360_config.json` so the host and LiDAR addresses match the
network, then run:

```bash
ros2 launch livox_ros_driver2 mid360_launch.py
```

Enable its point-cloud RViz view with:

```bash
ros2 launch livox_ros_driver2 mid360_launch.py rviz:=true
```

The default launch publishes:

- `/livox/lidar`: `sensor_msgs/msg/PointCloud2`
- `/livox/imu`: `sensor_msgs/msg/Imu`
- frame: `livox_frame`

`xfer_format=0` produces the Livox PointCloud2 layout:
`x`, `y`, `z`, `intensity`, `tag`, `line`, and per-point `timestamp`.

All launch files are installed under the standard ROS 2 package `launch`
directory.

## License

MIT. See `LICENSE.txt`.
