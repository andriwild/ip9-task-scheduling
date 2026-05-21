int32 time

# Structural category, matches des::RobotStateType. Plugin-specific flavours
# (search, accompany, clean, acquire, ...) all map to MISSION and are
# differentiated via the `name` field below.
uint8 IDLE = 0
uint8 MISSION = 1
uint8 CHARGING = 2
uint8 RETURNING = 3
uint8 state

# Plugin-specific state name (e.g. "search", "accompany", "clean", "acquire").
# Used by panels for tooltips/colors and by the metrics observer for buckets.
string name

# battery
float32 soc
float32 capacity
float32 low_threshold
