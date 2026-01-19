# MetricsReport.msg

# Robot State Durations (seconds)
int32 idle_time
int32 moving_time
int32 searching_time
int32 accompany_time
int32 charging_time

# Mission Statistics
int32 total_missions
int32 missions_on_time
int32 missions_late
int32 missions_failed
int32 missions_cancelled

# Performance Metrics
float32 avg_early_arrival
float32 avg_lateness

# Movement
float32 total_distance

# General
float32 utilization
