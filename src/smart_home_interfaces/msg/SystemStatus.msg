# SystemStatus.msg
# Real-time telemetry status published on /status

std_msgs/Header header
string state
string zone
float32 uptime_sec
string last_alert
float32 gas_ppm
