# GazeboRosRealsense

`GazeboRosRealsense` publishes simulated RealSense color, infrared and depth streams, camera information and a depth-derived point cloud. An optional depth model adds distance-dependent noise, missing pixels, blur and erosion.

Load `libRealsensePluginGazebo.so` inside the model. Implementation: [src/gazebo_ros_realsense.cpp](../../src/gazebo_ros_realsense.cpp).

## Interfaces

The model must contain the depth camera, color camera and infrared stereo sensor expected by the plugin. Sensor lookup uses the prefix `<namespace>/<camera_name><camera_suffix>_`, with suffixes `aligned_depth_to_color`, `color` and `infra_stereo`.

ROS topic names below are relative to the plugin's ROS node namespace:

| Output | Type | Purpose |
| --- | --- | --- |
| `<camera_name>/color/image_raw` | `sensor_msgs/msg/Image` | Color image. |
| `<camera_name>/infra1/image_raw`, `<camera_name>/infra2/image_raw` | `sensor_msgs/msg/Image` | Infrared stereo images. |
| `<camera_name>/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | `16UC1` depth in millimetres, with zero for invalid pixels. |
| Camera information paired with each image stream | `sensor_msgs/msg/CameraInfo` | Intrinsics calculated from image size and field of view. |
| `<camera_name>/lidar` | `sensor_msgs/msg/PointCloud2` | Depth-derived point cloud. |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Camera mounting and optical-frame transforms. |

The base implementation also publishes image messages over Gazebo transport. The ROS output topics depend on `camera_name` and the `<ros>` namespace; `camera_suffix` changes sensor lookup and frame names, not these topic suffixes.

## Configuration

| SDF field | Default | Meaning |
| --- | --- | --- |
| `camera_name` | `rgbd` | Camera-name prefix. |
| `camera_suffix` | Empty | Additional identifier for sensor lookup and frame names. |
| `namespace` | Empty | Prefix used for sensor lookup and frame names. |
| `parentFrameName` | `world` | Parent frame for the camera's mounting transform. |
| `x`, `y`, `z`, `roll`, `pitch`, `yaw` | Zero | Mounting translation in metres and rotation in radians. |
| `useRealistic` | `false` | Enables depth artifacts. |
| `imageScaling` | `4` when realistic | Depth-image upscaling factor. |
| `noisePerMeter`, `noiseMinDistance` | `0.2`, `4.0` | Depth-noise strength and minimum distance used to scale it. |
| `perlinEmptySpeed`, `perlinEmptyThreshold` | `0.1`, `0.8` | Evolution and threshold of the missing-pixel pattern. |
| `blurSize`, `erosionSize` | `15`, `5` | Image-kernel sizes for depth blur and erosion. |

Set image geometry and update rates in the parent sensors. Keep the plugin's `namespace`, sensor names, frame names and ROS namespace consistent with the model.

[Package overview](../../README.md)
