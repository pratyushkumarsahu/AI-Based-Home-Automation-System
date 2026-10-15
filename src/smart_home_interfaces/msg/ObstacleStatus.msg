# ObstacleStatus.msg
# Obstacle detection status published on /obstacle_status

std_msgs/Header header
bool is_blocked
float32 min_distance
float32 blocked_duration_sec
string description
