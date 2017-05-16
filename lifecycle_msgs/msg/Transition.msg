# Default values for primary states
# as described on
# http://design.ros2.org/articles/node_lifecycle.html

uint8 STATE_UNKNOWN = 0
uint8 STATE_UNCONFIGURED = 1
uint8 STATE_INACTIVE = 2
uint8 STATE_ACTIVE = 3
uint8 STATE_FINALIZED = 4

uint8 start_state
uint8 goal_state
