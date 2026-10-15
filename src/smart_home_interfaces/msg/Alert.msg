# Alert.msg
# Represents a suspicious activity, security alert, or hazard event

std_msgs/Header header
string alert_id
string alert_type
string severity
string zone
string description
float32 confidence
string snapshot_path
builtin_interfaces/Time timestamp
string identity
float32 gas_ppm
