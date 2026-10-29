# ArduPilotPlugin

`ArduPilotPlugin` exchanges simulator state and motor commands with ArduPilot SITL over UDP. It reads the vehicle's simulated state and IMU data, then publishes the received motor commands for the rotor plugins.

Load `libArdupilotPluginGazebo.so` inside the vehicle model. Implementation: [src/ardupilot_plugin.cpp](../../src/ardupilot_plugin.cpp).

## Interfaces

| Direction | Interface | Purpose |
| --- | --- | --- |
| Input | Gazebo `sensor_msgs::msgs::Imu` on `~/<model_name><imuTopic>` | Simulated inertial measurements. |
| Input | UDP on `listen_addr:fdm_port_in` | Motor commands from SITL. |
| Output | UDP to `fdm_addr:fdm_port_out` | Simulation time, vehicle state and sensor data for SITL. |
| Output | Gazebo `mav_msgs::msgs::CommandMotorSpeed` on `~/<model_name><motorSpeedCommandPubTopic>` | Command array consumed by the motor plugins. |

The plugin supplies the simulator connection. ArduPilot's DDS interface and the LUS `ap_api` node provide the separate ROS 2 connection.

## Configuration

| SDF field | Default or meaning |
| --- | --- |
| `fdm_addr` | SITL destination address, default `127.0.0.1`. |
| `listen_addr` | Local UDP bind address, default `127.0.0.1`. |
| `fdm_port_in` | Incoming motor-command port, default `9002`. |
| `fdm_port_out` | Outgoing simulator-state port, default `9003`. |
| `imuTopic` | Simulated IMU topic suffix. |
| `motorSpeedCommandPubTopic` | Motor-command output suffix. Match the rotor plugins' input. |
| `modelXYZToAirplaneXForwardZDown`, `gazeboXYZToNED` | Pose transforms between the model, Gazebo and firmware frame conventions. |
| `control` with `channel` attribute | Repeated channel entries. The channel selects the received motor value; entry order determines the output-array order. |
| `rangefinderName` | Name used to locate the optional model rangefinder. |
| `connectionTimeoutMaxCount` | Connection timeout setting; default `10`. |

The source also parses per-control type, multiplier, offset, filter and PID fields inherited from direct joint control. The active update path publishes motor commands; direct joint-force application is disabled. Configure physical rotor behavior in [GazeboMotorModel](../gazebo_motor_model/README.md).

Match the UDP ports to the SITL startup arguments. Keep channel order and coordinate transforms consistent with the vehicle model.

[Package overview](../../README.md)
