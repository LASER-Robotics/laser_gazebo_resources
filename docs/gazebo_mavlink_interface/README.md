# GazeboMavlinkInterface

`GazeboMavlinkInterface` connects simulated sensors and actuators to PX4 through MAVLink. It forwards Gazebo sensor measurements to the firmware and publishes the firmware's actuator commands for the motor plugins.

Load `libPx4PluginGazebo.so` inside the vehicle model. Implementation: [src/gazebo_mavlink_interface.cpp](../../src/gazebo_mavlink_interface.cpp).

## Interfaces

| Direction | Interface | Purpose |
| --- | --- | --- |
| Input | Gazebo Protobuf sensor topics | IMU, range, optical flow, magnetometer, pressure, ground truth, vision and other configured measurements. |
| Input/output | MAVLink over UDP, TCP or serial | Firmware sensor messages and actuator commands. |
| Output | Gazebo `mav_msgs::msgs::CommandMotorSpeed` on `~/<model_name><motorSpeedCommandPubTopic>` | Motor-command array for the rotor plugins. |

Match the motor-output topic to each motor plugin's `commandSubTopic`. Sensor topic suffixes must match their producing plugins. This is the simulator-to-firmware connection; PX4's ROS 2 telemetry and control connection is handled by the DDS client, agent and LUS hardware API.

## Configuration

| SDF field | Meaning |
| --- | --- |
| `robotNamespace` | Gazebo transport namespace. |
| `protocol_version` | MAVLink protocol version. |
| `mavlink_addr`, `mavlink_udp_port`, `mavlink_tcp_port` | Firmware connection address and network ports. |
| `use_tcp` | Selects TCP instead of UDP when serial is disabled. |
| `serialEnabled`, `serialDevice`, `baudRate` | Enables and configures serial transport. |
| `qgc_addr`, `qgc_udp_port`, `sdk_addr`, `sdk_udp_port` | Ground-station and SDK endpoints. |
| `hil_mode`, `hil_state_level` | Hardware-in-the-loop settings. |
| `enable_lockstep` | Synchronizes simulator stepping with firmware execution. |
| `motorSpeedCommandPubTopic` | Motor-command output suffix. |
| `imuSubTopic`, `lidarSubTopic`, `opticalFlowSubTopic` | IMU, rangefinder and optical-flow input suffixes. |
| `magSubTopic`, `baroSubTopic`, `groundtruthSubTopic` | Magnetometer, pressure and ground-truth input suffixes. |
| `visionSubTopic`, `irlockSubTopic`, `targetGpsSubTopic`, `arucoMarkerSubTopic` | Vision, target and marker inputs. |
| `send_odometry`, `send_vision_estimation` | Selects the MAVLink messages used to forward vision estimates. |
| `control_channels/channel` | Per-channel index, scaling, offsets, armed/disarmed zero positions and joint-control settings. |

For lockstep, the world physics `real_time_update_rate` must be a multiple of 250 Hz and `max_step_size` must equal its reciprocal. `PX4_SIM_SPEED_FACTOR` selects a positive simulation speed factor. Use distinct connection ports for separate vehicle instances and match each to the firmware startup configuration.

[Package overview](../../README.md)
