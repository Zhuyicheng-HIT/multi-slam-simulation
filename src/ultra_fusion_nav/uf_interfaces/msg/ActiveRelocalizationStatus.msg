std_msgs/Header header

uint8 NORMAL_NAVIGATION=0
uint8 HOLD=1
uint8 ACTIVE_RELOCALIZATION=2
uint8 RECOVERY_VALIDATION=3
uint8 RESUME=4
uint8 FAILSAFE=5

uint8 state
string action
bool request_active
bool motion_authorized
bool epoch_committed
uint64 transaction_id
uint32 candidate_id
uint32 yaw_views_completed
uint32 safe_motion_steps_completed
uint32 failure_count
float32 elapsed_s
string reason
