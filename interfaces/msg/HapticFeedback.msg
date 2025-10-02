# Haptic feedback message

# Header
std_msgs/Header header

# Feedback pattern
uint8 pattern
uint8 PATTERN_SHORT_PULSE = 0
uint8 PATTERN_LONG_PULSE = 1
uint8 PATTERN_DOUBLE_PULSE = 2
uint8 PATTERN_CONTINUOUS = 3
uint8 PATTERN_WARNING = 4

# Intensity (0-100)
uint8 intensity

# Duration in milliseconds
uint32 duration_ms
