# GazeboRosVelodyneLaser

`GazeboRosVelodyneLaser` converts Gazebo ray or GPU-ray scans into ROS 2 point clouds. It supports regular angular sampling or explicit horizontal and vertical scan angles.

Load `libVelodynePluginGazebo.so` inside the ray sensor. Implementation: [src/gazebo_ros_velodyne_laser.cpp](../../src/gazebo_ros_velodyne_laser.cpp).

## Interfaces

The plugin subscribes to the parent sensor's Gazebo `LaserScanStamped` topic. It publishes `sensor_msgs/msg/PointCloud2` on `topicName`, with fields `x`, `y`, `z`, `intensity` and `ring`, plus mounting transforms on `/tf_static` (`tf2_msgs/msg/TFMessage`).

Cloud timestamps come from the Gazebo scan. The cloud frame uses `lidar_frame_name`. IMU-frame settings define the mounting transform; IMU measurements come from a separate sensor.

## Configuration

| SDF field | Meaning |
| --- | --- |
| `topicName` | ROS cloud topic; default `/points`. |
| `robot_namespace` | Prefix used in the mounting transforms; default `uav1`. |
| `parent_frame_name`, `frameName`, `lidar_frame_name`, `imu_frame_name` | Parent, sensor, LiDAR and IMU frame names. |
| `sensor_x`, `sensor_y`, `sensor_z`, `sensor_roll`, `sensor_pitch`, `sensor_yaw` | Sensor pose relative to its parent. |
| `lidar_x`, `lidar_y`, `lidar_z`, `lidar_roll`, `lidar_pitch`, `lidar_yaw` | LiDAR pose relative to the sensor frame. |
| `imu_x`, `imu_y`, `imu_z`, `imu_roll`, `imu_pitch`, `imu_yaw` | IMU pose relative to the sensor frame. |
| `min_range`, `max_range` | Additional range clipping in metres, bounded by the parent sensor's range. |
| `min_intensity` | Minimum retained point intensity; omitted means no intensity clipping. |
| `gaussianNoise` | Standard deviation of added range noise; default `0.0`. |
| `scan/horizontal/angles`, `scan/vertical/angles` | Optional lists of scan angles in radians within the parent sensor's angular bounds. |

Without explicit angle lists, samples are distributed evenly over the sensor's scan bounds. Configure the parent sensor's sample counts, range and update rate alongside these plugin settings.

[Package overview](../../README.md)
