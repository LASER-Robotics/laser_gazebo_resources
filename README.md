# LASER Gazebo Resources

`laser_gazebo_resources` provides the Gazebo Classic plugins, models and worlds used by the LASER UAV System (LUS). `laser_uav_simulation` loads these resources when it builds and spawns a vehicle model.

The plugins simulate sensors, rotor forces and communication with SITL firmware. They are shared libraries loaded from SDF model files, rather than standalone ROS executables.

## Plugins and interfaces

| Library | Role and interfaces |
| --- | --- |
| `libLivoxPluginGazebo.so` | Uses CSV scan patterns to simulate Livox LiDAR. Publishes a ROS point cloud or custom Livox-style message and sensor transforms. |
| `libMotorPluginGazebo.so` | Receives motor commands through Gazebo transport, applies rotor forces and moments, and publishes motor speed. Supports the coefficient-based model and an optional blade-element model. |
| `libPx4PluginGazebo.so` | Exchanges simulator sensor data and actuator commands with PX4 through MAVLink. |
| `libArdupilotPluginGazebo.so` | Connects the simulated vehicle to ArduPilot SITL. |
| `libImuPluginGazebo.so` | Publishes `sensor_msgs/msg/Imu`; the plugin's `~/out` topic is remapped by the model. |
| `libP3dPluginGazebo.so` | Publishes `nav_msgs/msg/Odometry` for simulated ground truth. |
| `libRealsensePluginGazebo.so` | Simulates RealSense camera streams. |
| `libLidarPluginGazebo.so`, `libVelodynePluginGazebo.so` | Additional LiDAR simulation plugins. |
| `liblink_static_tf_publisher.so` | Publishes the fixed transform between two model links on `/tf_static`. |

Topic names, vehicle namespaces, sensor poses and update rates are configured in the model SDF. Motor commands and Protobuf sensor data use Gazebo transport; these are separate from ROS topics.

## Configuration

### Livox LiDAR

The `LivoxPointsPlugin` loads its scan pattern from a CSV in [resources/](resources/). Available patterns include Mid-360, Mid-40, Mid-70, Avia, Horizon and Tele.

| SDF field | Meaning |
| --- | --- |
| `csv_file_name` | Scan-pattern CSV path. |
| `robot_namespace` | ROS namespace prefix for sensor output. |
| `topic_lidar_name` | Point-cloud topic name. |
| `parent_frame_name`, `sensor_frame_name`, `lidar_frame_name`, `imu_frame_name` | Frames used for the sensor transforms. |
| `sensor_*`, `lidar_*` | Sensor and LiDAR position and rotation relative to the model frames. |
| `samples` | Number of scan-pattern samples processed per update. |
| `downsample` | Scan-pattern sampling stride; at least 1. |
| `publish_pointcloud_type` | `0`: `sensor_msgs/msg/PointCloud`; `1`: XYZ `PointCloud2`; `2`: Livox-field `PointCloud2`; `3`: `laser_gazebo_resources/msg/CustomMsg`. |
| `ray.range.min`, `ray.range.max` | Simulated minimum and maximum range in metres. |

`CustomMsg` contains the scan's point data using the package's `CustomPoint` type. Select a format accepted by the downstream LiDAR estimator.

### Motor model

| SDF field | Meaning |
| --- | --- |
| `jointName`, `linkName`, `motorNumber`, `turningDirection` | Rotor joint, body link, command index and rotation direction. |
| `commandSubTopic`, `motorSpeedPubTopic`, `windSubTopic` | Gazebo transport command, speed-feedback and wind topics. |
| `motorConstant`, `momentConstant` | Thrust and reaction-torque coefficients. |
| `timeConstantUp`, `timeConstantDown` | Spin-up and spin-down time constants in seconds. |
| `maxRotVelocity` | Maximum rotor angular velocity. |
| `rotorVelocitySlowdownSim` | Scale used to slow the rendered rotor rotation. |
| `rotorDragCoefficient`, `rollingMomentCoefficient` | Rotor drag and rolling-moment coefficients. |
| `motorQuadraticA`, `motorQuadraticB` | Normalized-command motor-curve coefficients. |
| `commandDeadband`, `commandMax`, `reversible` | Command range and reversible-rotation settings. |
| `useBem` | Enables the blade-element model and its required propeller/aerodynamic settings. |

The blade-element model additionally uses `commandMode`, `bemRho`, `propDiameterInches`, `propPitchInches`, `propChordM`, `propNb`, `bemA`, `bemCd0`, `bemXRoot`, `bemRadialStations`, `bemAzimuthStations` and `bemCorrectPitchInterpolation`. These describe command interpretation, air density, propeller geometry and the numerical integration. Use a complete calibrated set; the plugin rejects missing or invalid required values.

### Static link transforms

The static-TF plugin reads `robotNamespace`, `parentLink` and `childLink`. The links must exist in the model. It calculates their relative pose when loaded and publishes a static transform; use it for links whose relationship remains fixed.

## Models and worlds

[models/](models/) contains a Livox sensor resource, a tree model and a wall with an opening. [worlds/](worlds/) includes `custom_empty.world`, `forest.world`, `hole.world`, `walls.world`, `c.world` and `u.world`.

The [proto_msg/](proto_msg/) directory defines Gazebo transport messages for motor commands, IMU, GPS, odometry, range, wind and other simulator data.

## Build and use

Build from a ROS 2 workspace with the Gazebo Classic dependencies installed:

```bash
colcon build --packages-up-to laser_gazebo_resources
source install/setup.bash
```

Load plugins from the UAV model SDF or start a world through `laser_uav_simulation`. The LUS installer sets up the simulation environment; the package also installs an environment hook for its resources.
