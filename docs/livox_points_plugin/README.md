# LivoxPointsPlugin

`LivoxPointsPlugin` simulates a Livox LiDAR by casting rays along a CSV scan pattern. Its clouds can feed the LUS LiDAR-odometry and mapping packages.

Load `libLivoxPluginGazebo.so` inside a Gazebo ray sensor. Implementation: [src/livox_points_plugin.cpp](../../src/livox_points_plugin.cpp).

## Interfaces

The plugin reads scan geometry and range limits from the sensor SDF and publishes one cloud format selected by `publish_pointcloud_type`:

| Value | ROS output type |
| --- | --- |
| `0` | `sensor_msgs/msg/PointCloud` |
| `1` | `sensor_msgs/msg/PointCloud2` with XYZ points. |
| `2` | `sensor_msgs/msg/PointCloud2` with Livox point fields. |
| `3` | `laser_gazebo_resources/msg/CustomMsg`, containing `CustomPoint` values. |

`topic_lidar_name` selects the output topic, prefixed with `robot_namespace` when supplied. Static mounting transforms are published on `/tf_static` (`tf2_msgs/msg/TFMessage`). The IMU-frame settings define a transform; IMU measurements come from a separate sensor plugin.

## Configuration

| SDF field | Meaning |
| --- | --- |
| `csv_file_name` | Filename in the package's installed `resources/` directory, such as `mid360-real-centr.csv`. |
| `robot_namespace` | Prefix used for the cloud topic and frame names. |
| `topic_lidar_name` | Cloud output name within that prefix. |
| `parent_frame_name`, `sensor_frame_name`, `lidar_frame_name`, `imu_frame_name` | Parent and sensor frame names. |
| `sensor_x`, `sensor_y`, `sensor_z`, `sensor_roll`, `sensor_pitch`, `sensor_yaw` | Mounting pose of the sensor relative to its parent. |
| `lidar_x`, `lidar_y`, `lidar_z`, `lidar_roll`, `lidar_pitch`, `lidar_yaw` | LiDAR pose relative to the sensor frame. |
| `imu_x`, `imu_y`, `imu_z`, `imu_roll`, `imu_pitch`, `imu_yaw` | IMU pose relative to the sensor frame. |
| `samples` | Scan-pattern samples processed per update. |
| `downsample` | Pattern sampling stride; values below 1 are clamped to 1. |
| `publish_pointcloud_type` | Cloud format from the table above. |
| `ray/range/min`, `ray/range/max` | Sensor range limits in metres. |

Translations use metres and rotations use radians. Choose a point format accepted by the downstream estimator and set the sensor's update rate for the intended scan frequency.

[Package overview](../../README.md)
