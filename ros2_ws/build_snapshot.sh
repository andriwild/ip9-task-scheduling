#!/bin/zsh
#
# BEGIN AI-assisted: generated with Claude Code (Anthropic),
# reviewed and adapted by the author.
#
#   ./build_snapshot.sh          full rebuild, recomputes every distance with Nav2 (planner.sh must run)
#   ./build_snapshot.sh rooms    rooms, types and footprints from the DB, distance matrix kept
#
source /opt/ros/jazzy/setup.zsh
WS_DIR=${0:A:h}
source $WS_DIR/install/setup.zsh

MODE=build
if [ "${1:-}" = "rooms" ] || [ "${1:-}" = "build_rooms" ]; then
  MODE=build_rooms
fi

RCUTILS_COLORIZED_OUTPUT=1 \
RCUTILS_LOGGING_MIN_SEVERITY=INFO \
ros2 launch event_system_bringup bringup.launch.py mode:=$MODE log_level:=INFO
