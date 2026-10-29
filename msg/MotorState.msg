# Diagnostics for experiments only. Never fed back to the actuator command path.
# stamp is Gazebo simulation time; frame_id is the scoped rotor link name.
std_msgs/Header header
int32 motor_id
bool use_bem
# Accepted active schedule; 0 after reset (use nonzero request IDs).
uint64 request_id
float64 failure_start_time
bool scheduled
bool transitioning
# Result of the last processed command. Acceptance is not proof of application:
# inspect scheduled, efficiency and the timestamp. Rejections retain the schedule.
uint64 last_request_id
bool request_accepted
string request_status
# Same-step input snapshot. Units depend on command_mode.
string command_mode
float64 command_received
float64 command_limited
float64 efficiency
# Physical model speed magnitude, rad/s, before the visualization slowdown.
float64 omega
# Axial thrust in rotor +Z, N. Reaction torque in rotor +Z, N m (signed).
# Nominal means healthy output at the SAME current omega and inflow.
float64 nominal_thrust
float64 actual_thrust
float64 nominal_reaction_torque
float64 actual_reaction_torque
# False when a load was skipped, e.g. non-finite BEM output or missing parent.
bool loads_valid
bool bem_solver_fallback
bool legacy_failure_active
