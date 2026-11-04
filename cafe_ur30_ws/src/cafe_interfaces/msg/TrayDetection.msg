# One detected tray, expressed in header.frame_id (normally "world").
std_msgs/Header header
uint32 id              # persistent track id (stays the same while the tray is tracked)
string color           # "red" | "green" | "blue"
geometry_msgs/Pose pose  # tray centre; orientation = rotation about +Z, X axis along the LONG side
float32 yaw            # long-axis direction in radians, normalised to [-pi/2, pi/2)
float32 length         # metres (long side)
float32 width          # metres (short side) -> the gripper closes across this
float32 height         # metres (top surface above table)
float32 confidence     # 0..1 (grows with consecutive sightings)
bool stable            # true once the pose has stopped moving (safe to pick)
