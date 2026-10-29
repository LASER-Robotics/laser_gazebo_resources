# Experiment command. Does not modify the normal actuator command channel.
# Motor IDs follow SDF motorNumber (0..3 on the quadrotors; rotor 3 has ID 2).
uint8 ABRUPT=0
uint8 LINEAR=1
int32 motor_id
# Absolute Gazebo simulation seconds. Zero means the next available physics step.
float64 failure_start_time
float64 target_efficiency 1.0
uint8 failure_type
# Seconds; strictly positive for LINEAR and zero for ABRUPT.
float64 transition_duration
# Use distinct IDs to correlate acceptance/rejection with MotorState.
uint64 request_id
