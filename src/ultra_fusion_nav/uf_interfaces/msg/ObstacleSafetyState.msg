std_msgs/Header header

uint8 CLEAR=0
uint8 CAUTION=1
uint8 BRAKE=2
uint8 HOVER_REQUIRED=3

uint8 state
bool raw_sensor_healthy
bool fail_closed
float32 nearest_clearance_m
float32 path_clearance_m
float32 time_to_collision_s
float32 stopping_distance_m
float32 raw_sensor_age_s
string reason
