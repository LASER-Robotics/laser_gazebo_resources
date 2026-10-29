# GazeboMotorModel

`GazeboMotorModel` applies rotor thrust, reaction torque, drag and rolling moments to the simulated vehicle. It includes motor spin-up and spin-down dynamics, a quadratic motor curve and an optional blade-element model (BEM).

Load `libMotorPluginGazebo.so` once per rotor inside the model. Implementation: [src/gazebo_motor_model.cpp](../../src/gazebo_motor_model.cpp).

## Interfaces

| Direction | Gazebo transport interface | Purpose |
| --- | --- | --- |
| Input | `mav_msgs::msgs::CommandMotorSpeed` on `~/<model_name><commandSubTopic>` | Motor-command array; this instance reads the zero-based `motorNumber` element. |
| Input | `physics_msgs::msgs::Wind` on the configured `windSubTopic` | Wind velocity used for aerodynamic loads. |
| Input | `gazebo::msgs::Int` on `/gazebo/motor_failure_num` | One-based motor number to fail; `0` restores all motors. |
| Output | Simulated link forces, torques and rotor-joint velocity | Vehicle motion and rotor animation. |

Motor-speed publication is disabled in the implementation. Setting `motorSpeedPubTopic` does not produce a feedback stream. These interfaces use Gazebo transport rather than ROS topics.

## Configuration

| SDF field | Meaning |
| --- | --- |
| `robotNamespace` | Gazebo transport namespace. |
| `jointName`, `linkName` | Existing rotor joint and link. Both are required. |
| `motorNumber` | Required zero-based index in the command array. |
| `turningDirection` | Required rotation direction: `cw` or `ccw`. |
| `commandSubTopic`, `windSubTopic` | Motor-command suffix and wind topic. |
| `motorConstant`, `momentConstant` | Thrust coefficient and reaction-torque ratio for the coefficient-based model. |
| `motorQuadraticA`, `motorQuadraticB` | Coefficients of the command-to-thrust curve. |
| `timeConstantUp`, `timeConstantDown` | Positive spin-up and spin-down time constants in seconds. |
| `maxRotVelocity` | Maximum rotor angular velocity in rad/s. |
| `rotorVelocitySlowdownSim` | Positive scale factor used to slow rotor animation. |
| `rotorDragCoefficient`, `rollingMomentCoefficient` | Rotor drag and rolling-moment coefficients. |
| `commandDeadband`, `commandMax` | Motor-command deadband and maximum throttle command. |
| `reversible` | Allows signed input in the applicable command modes. |
| `joint_control_pid` | Parsed PID settings (`p`, `i`, `d`, `iMax`, `iMin`, `cmdMax`, `cmdMin`). The active rotor update sets joint velocity directly and does not use this PID. |
| `useBem` | Enables the blade-element model. |

With `useBem=false`, the input passes through the motor time response and quadratic command-to-thrust curve. Match the curve and thrust coefficient to the motor/propeller setup.

### Blade-element model

With `useBem=true`, provide every field below. The plugin rejects missing required settings.

| SDF field | Meaning |
| --- | --- |
| `commandMode` | `throttle`: motor-curve input; `omega`: rad/s; `thrust`: desired static thrust in newtons. |
| `bemRho` | Air density in kg/m³. |
| `propDiameterInches`, `propPitchInches` | Propeller diameter and pitch in inches. |
| `propChordM`, `propNb` | Blade chord in metres and number of blades. |
| `bemA`, `bemCd0` | Lift-curve slope and profile-drag coefficient. |
| `bemXRoot` | Fractional radius where blade integration begins, from 0 inclusive to 1 exclusive. |
| `bemRadialStations`, `bemAzimuthStations` | Numerical integration counts: at least 1 radial and 4 azimuthal stations. |
| `bemCorrectPitchInterpolation` | Selects pitch interpolation relative to the configured blade root. |

Throttle mode also requires explicit `motorQuadraticA` and `motorQuadraticB`. Use a complete calibrated parameter set for the propeller and motor command convention.

[Package overview](../../README.md)
