# LidarPlugin

`LidarPlugin` turns the first ray of a Gazebo ray sensor into a rangefinder measurement for the firmware bridge. It reports distance, sensor orientation and an estimated signal quality.

Load `libLidarPluginGazebo.so` inside a ray sensor. Implementation: [src/gazebo_lidar_plugin.cpp](../../src/gazebo_lidar_plugin.cpp).

## Interfaces

The plugin reads the parent ray sensor and publishes the Protobuf type `sensor_msgs::msgs::Range` through Gazebo transport. This is separate from the ROS type `sensor_msgs/msg/Range`.

The local transport topic is formed as `~/<model_name><topic>`. The PX4 MAVLink plugin can consume this measurement and forward it to the firmware.

## Configuration

| SDF field | Meaning |
| --- | --- |
| `robotNamespace` | Gazebo transport namespace. |
| `topic` | Range-output suffix, including the separator needed after the model name. If omitted, the sensor model name is used. |
| `min_distance`, `max_distance` | Reported range limits in metres, constrained by the plugin's sensor bounds. |
| `simulate_fog` | Enables the fog approximation; default is `false`. Returns beyond 2 m are replaced by a noisy reading near 2 m. |

The sensor SDF sets the actual ray geometry and update rate. Values below the minimum are clipped; ranges above the maximum or infinite returns are reported as zero. This plugin produces a range measurement, rather than a point cloud.

[Package overview](../../README.md)
