# MetricsReport.msg

# Robot State Durations (seconds)
int32 idle_time
int32 moving_time
int32 searching_time
int32 accompany_time
int32 charging_time
int32 talk_time

# Mission Statistics — scheduled missions (have a deadline)
int32 scheduled_total
int32 scheduled_on_time
int32 scheduled_late
int32 scheduled_failed
int32 scheduled_cancelled
int32 scheduled_rejected

# Mission Statistics — background missions (no deadline; how many of the
# registered ones got executed)
int32 background_total
int32 background_completed
int32 background_failed
int32 background_cancelled

# Mission Statistics — interrupt missions (ad-hoc, all that arrived)
int32 interrupt_total
int32 interrupt_completed

# Performance Metrics
float32 avg_early_arrival
float32 avg_lateness
int32 min_lateness
int32 max_lateness
float32 rejected_rate

# Movement
float32 total_distance

# Energy (Ah)
float32 energy_total_consumed_ah
float32 energy_idle_ah
float32 energy_searching_ah
float32 energy_accompany_ah
float32 energy_talk_ah
float32 energy_charging_ah

# General
float32 utilization
float32 idle_percent
float32 charging_percent
float32 returning_percent
