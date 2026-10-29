# LASER Gazebo Resources

`laser_gazebo_resources` provides the Gazebo Classic plugins, models and worlds used by the LASER UAV System (LUS). `laser_uav_simulation` loads these resources when it generates and spawns a vehicle model.

The plugins simulate sensors, rotor forces and communication with SITL firmware. Gazebo loads them as shared libraries from the model's SDF description.

## Build

From a ROS 2 workspace with the Gazebo Classic dependencies installed:

```bash
colcon build --packages-up-to laser_gazebo_resources
source install/setup.bash
```

## Use in a simulation

Load model plugins inside the SDF `<model>` element and sensor plugins inside the corresponding `<sensor>` element. Each plugin's page lists its library filename, inputs, outputs and configuration.

Libraries are installed under `lib/`; models, worlds and scan patterns are installed under `share/laser_gazebo_resources/`. The package's environment hook adds its model and resource directories to the Gazebo search paths.

For complete vehicle launch sessions, use [laser_uav_simulation](https://github.com/LASER-Robotics/laser_uav_simulation). ROS namespaces, topic remappings and frame names come from the model configuration. Gazebo transport topics and ROS topics use separate communication systems.

## Plugins

| Plugin | Library | Role |
| --- | --- | --- |
| [LivoxPointsPlugin](docs/livox_points_plugin/README.md) | `libLivoxPluginGazebo.so` | LiDAR scans generated from Livox CSV patterns. |
| [GazeboMotorModel](docs/gazebo_motor_model/README.md) | `libMotorPluginGazebo.so` | Rotor forces, moments, motor dynamics and failure injection. |
| [GazeboMavlinkInterface](docs/gazebo_mavlink_interface/README.md) | `libPx4PluginGazebo.so` | Sensor and actuator communication with PX4. |
| [ArduPilotPlugin](docs/ardupilot_plugin/README.md) | `libArdupilotPluginGazebo.so` | Sensor and actuator communication with ArduPilot SITL. |
| [GazeboRosImuSensor](docs/gazebo_ros_imu_sensor/README.md) | `libImuPluginGazebo.so` | IMU measurements published to ROS 2. |
| [GazeboRosP3D](docs/gazebo_ros_p3d/README.md) | `libP3dPluginGazebo.so` | Ground-truth odometry for a model link. |
| [GazeboRosRealsense](docs/gazebo_ros_realsense/README.md) | `libRealsensePluginGazebo.so` | Color, infrared and depth images, camera information and point clouds. |
| [LidarPlugin](docs/gazebo_lidar_plugin/README.md) | `libLidarPluginGazebo.so` | Rangefinder measurements for the firmware bridge. |
| [GazeboRosVelodyneLaser](docs/gazebo_ros_velodyne_laser/README.md) | `libVelodynePluginGazebo.so` | Point clouds from ray or GPU-ray sensors. |
| [LinkStaticTFPublisher](docs/link_static_tf_publisher/README.md) | `liblink_static_tf_publisher.so` | Fixed transforms between model links. |

## Models and worlds

| Directory | Contents |
| --- | --- |
| [models/](models/) | Livox sensor resources, a tree model and a wall with an opening. |
| [worlds/](worlds/) | `custom_empty.world`, `forest.world`, `hole.world`, `walls.world`, `c.world` and `u.world`. |
| [resources/](resources/) | Livox scan patterns for Mid-360, Mid-40, Mid-70, Avia, Horizon and Tele. |

## Messages

The [msg/](msg/) directory defines the ROS 2 `CustomMsg` and `CustomPoint` types used by the Livox plugin. These types describe a scan and its individual points.

[proto_msg/](proto_msg/) contains Protobuf definitions for Gazebo transport, including motor commands, IMU, GPS, odometry, range and wind. They support communication between simulation plugins and firmware bridges.
