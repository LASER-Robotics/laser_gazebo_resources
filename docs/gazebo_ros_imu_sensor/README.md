# GazeboRosImuSensor

`GazeboRosImuSensor` publishes the orientation, angular velocity and linear acceleration of a Gazebo IMU sensor to ROS 2. It also fills gyroscope and accelerometer covariance diagonals from the sensor's noise model.

Load `libImuPluginGazebo.so` inside an IMU sensor. Implementation: [src/gazebo_ros_imu_sensor.cpp](../../src/gazebo_ros_imu_sensor.cpp).

## Interfaces

The plugin reads the parent sensor and publishes `sensor_msgs/msg/Imu` on `~/out`. Use the plugin's `<ros>` settings to choose the node namespace and remap that topic. Message timestamps come from the sensor's last simulation update.

## Configuration

| SDF field | Default | Meaning |
| --- | --- | --- |
| `robot_namespace` | `uav1` | Frame-name prefix. |
| `parent_frame_name` | `sensor` | Frame-name component; the message frame is `<robot_namespace>/<parent_frame_name>/imu`. |
| `initial_orientation_as_reference` | `false` | With `false`, orientation uses the world reference. `true` uses the sensor's initial reference and produces a deprecation warning. |

Configure update rate and noise in the parent IMU sensor. `robot_namespace` controls the frame label; the `<ros><namespace>` element controls the ROS node namespace.

For example, inside the sensor:

```xml
<plugin name="imu" filename="libImuPluginGazebo.so">
  <ros>
    <namespace>/uav1</namespace>
    <remapping>~/out:=imu/data</remapping>
  </ros>
  <robot_namespace>uav1</robot_namespace>
  <parent_frame_name>fcu</parent_frame_name>
</plugin>
```

[Package overview](../../README.md)
