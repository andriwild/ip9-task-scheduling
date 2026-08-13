int32 time

uint8 SIMULATION_START = 0
uint8 SIMULATION_END = 1
uint8 STOP_DRIVE = 2
uint8 MISSION_COMPLETE = 3
uint8 MISSION_DISPATCH = 4
uint8 ABORT_SEARCH = 7
uint8 START_ACCOMPANY = 10
uint8 ARRIVED_ACCOMPANY = 11
uint8 START_DRIVE = 12
uint8 BATTERY_FULL = 13
uint8 RESET = 14
uint8 PERSON_TRANSITION = 15
uint8 PERSON_ARRIVED = 16
uint8 PERSON_DEPARTURE = 17
uint8 PERSON_ACCOMPANY_DEPARTURE = 22
uint8 PERSON_ACCOMPANY_ARRIVED = 23
uint8 MISSION_START = 18
uint8 APPOINTMENT_END = 19
uint8 DATA_ACQUISITION_START = 24
uint8 DATA_ACQUISITION = 25
uint8 CLEAN_START = 26
uint8 CLEAN = 27
uint8 ORDER_ARRIVAL = 28
uint8 INFORMATION_START = 29
uint8 INFORMATION = 30
uint8 CHARGE_MISSION_START = 31
uint8 CHARGE_MISSION = 32
uint8 CHARGE_PHASE_TRANSITION = 33
uint8 PERSON_ROOM_ARRIVED = 34
uint8 BACKGROUND_RELEASE = 35
uint8 SCAN = 36
uint8 CONVERSATION_START = 37
uint8 CONVERSATION_END = 38

uint8 type

bool is_driving
bool is_charging
string label
string color
# -1 if the event is not associated with a mission (e.g. system events,
# person transitions). Otherwise the IOrder.id of the active mission.
int32 mission_id
