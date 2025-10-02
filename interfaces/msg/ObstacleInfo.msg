# Obstacle information message

# Header
std_msgs/Header header

# Obstacle position
geometry_msgs/Point position

# Obstacle dimensions
float32 width
float32 height
float32 depth

# Distance to obstacle
float32 distance

# Obstacle type
uint8 type
uint8 TYPE_UNKNOWN = 0
uint8 TYPE_PERSON = 1
uint8 TYPE_VEHICLE = 2
uint8 TYPE_OBJECT = 3
uint8 TYPE_WALL = 4

# Confidence level (0.0 - 1.0)
float32 confidence
