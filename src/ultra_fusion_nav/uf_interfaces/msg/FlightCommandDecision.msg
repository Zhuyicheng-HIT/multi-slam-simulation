std_msgs/Header header

uint8 RELEASE=0
uint8 FORWARD=1
uint8 HOLD=2
uint8 LAND=3
uint8 RETURN=4

uint8 action
string owner
string reason
bool fail_closed
bool setpoint_published
float32 selected_intent_age_s
