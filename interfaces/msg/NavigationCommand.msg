# Navigation command message

# Header
std_msgs/Header header

# Command type
uint8 command
uint8 CMD_GOTO = 0
uint8 CMD_STOP = 1
uint8 CMD_PAUSE = 2
uint8 CMD_RESUME = 3
uint8 CMD_CANCEL = 4

# Target pose (for GOTO command)
geometry_msgs/PoseStamped target_pose

# Additional parameters
string description
