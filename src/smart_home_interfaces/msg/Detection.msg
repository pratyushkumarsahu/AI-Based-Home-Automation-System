# Detection.msg
# Represents a single AI detection event with Family/Outsider classification

string class_name
float32 confidence
int32 xmin
int32 ymin
int32 xmax
int32 ymax
builtin_interfaces/Time timestamp
string zone
string identity
bool is_family
