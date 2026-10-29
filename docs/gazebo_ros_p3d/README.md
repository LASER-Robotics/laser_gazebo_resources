# GazeboRosP3D

`GazeboRosP3D` publishes a model link's pose and velocity as odometry. It provides ground-truth feedback for simulation and estimator comparisons, with optional pose offsets and velocity noise.

Load `libP3dPluginGazebo.so` inside the model. Implementation: [src/gazebo_ros_p3d.cpp](../../src/gazebo_ros_p3d.cpp).

## Interfaces

The input is the configured link's simulated state. The output is `nav_msgs/msg/Odometry` on `odom`, remappable through the plugin's `<ros>` settings. The message uses simulation time, `frame_name` as its reference frame and the tracked link name as `child_frame_id`.

## Configuration

| SDF field | Default | Meaning |
| --- | --- | --- |
| `body_name` | Required | Existing model link to track. |
| `frame_name` | `world` | Reference frame. A link name selects a reference link in the same model; `world` and `map` use world values. |
| `update_rate` | `0.0` | Maximum publication rate in Hz; zero allows every eligible physics update. |
| `xyz_offset` | `0 0 0` | Position offset in metres. |
| `rpy_offset` | `0 0 0` | Orientation offset in radians. |
| `local_twist` | `false` | Reports velocity in the tracked link's local frame when enabled. |
| `gaussian_noise` | `0.0` | Standard deviation of noise added to each velocity component. |

The older `xyz_offsets` and `rpy_offsets` names are accepted with warnings; the singular forms take precedence. If a non-world `frame_name` does not identify a model link, the plugin falls back to world values. It publishes only while a subscriber is connected.

For example, inside a model containing `base_link`:

```xml
<plugin name="ground_truth" filename="libP3dPluginGazebo.so">
  <ros>
    <namespace>/uav1</namespace>
    <remapping>odom:=ground_truth/odometry</remapping>
  </ros>
  <body_name>base_link</body_name>
  <frame_name>world</frame_name>
  <update_rate>100.0</update_rate>
</plugin>
```

[Package overview](../../README.md)
