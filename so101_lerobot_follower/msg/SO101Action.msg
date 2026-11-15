# LeRobot SO-101 action in the default use_degrees=true representation.
# order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
# values[0..4] are degrees; values[5] is RANGE_0_100 percent.
# Header timestamp is leader ROS time. sequence increments once per published action.
std_msgs/Header header
uint32 sequence
string[6] joint_names
float64[6] values
