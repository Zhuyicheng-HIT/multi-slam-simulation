# A read-only copy of one completed FAST-LIO posterior. A clean-scan gateway
# may use this state only for a later scan. It is never a command or feedback
# message and it does not authorize changing FAST-LIO's production input.
std_msgs/Header header

string map_frame
string body_frame
uint64 scan_sequence
uint32 reset_counter
bool valid

float64[3] position
float64[4] orientation_xyzw
float64[3] velocity_map
float64[3] accel_bias
float64[3] gyro_bias
