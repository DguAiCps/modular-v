# Module status message
# Provides health and state information for a module

# Header
std_msgs/Header header

# Module identification
string module_name
string module_version

# State information
uint8 state
uint8 STATE_UNINITIALIZED = 0
uint8 STATE_INITIALIZED = 1
uint8 STATE_RUNNING = 2
uint8 STATE_PAUSED = 3
uint8 STATE_ERROR = 4
uint8 STATE_TERMINATED = 5

# Health status
bool is_healthy
string health_message

# Performance metrics
float32 cpu_usage       # Percentage (0-100)
float32 memory_usage     # Percentage (0-100)
float32 processing_time  # milliseconds

# Additional statistics
uint64 messages_processed
uint64 errors_count
float32 uptime          # seconds