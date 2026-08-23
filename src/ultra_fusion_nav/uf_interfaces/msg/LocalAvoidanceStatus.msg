std_msgs/Header header

uint8 NAVIGATING=0
uint8 PATH_BLOCKED=1
uint8 BRAKE_HOLD=2
uint8 REPLAN=3
uint8 TRAJECTORY_VERIFY=4
uint8 RESUME=5
uint8 HOVER_REQUIRED=6

uint8 state
bool healthy
bool fail_closed
uint32 replan_count
uint32 consecutive_replans
uint32 path_points
float32 planning_latency_ms
float32 trajectory_clearance_m
string reason
