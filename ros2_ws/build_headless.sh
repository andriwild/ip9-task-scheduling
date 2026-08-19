#!/usr/bin/env bash
# Builds the ROS-free simulator into build-headless/.
#
#   ./build_headless.sh          incremental
#   ./build_headless.sh --clean  from scratch
set -euo pipefail

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$WS_DIR/build-headless"

[ "${1:-}" = "--clean" ] && rm -rf "$BUILD_DIR"

cmake -S "$WS_DIR/src/event_system_core/src" -B "$BUILD_DIR" \
      -DCMAKE_BUILD_TYPE="${BUILD_TYPE:-Release}"
cmake --build "$BUILD_DIR" -j"$(nproc)"

echo
echo "built: $BUILD_DIR/des_headless"
ldd "$BUILD_DIR/des_headless" | grep -qE 'rclcpp|rcl_|rmw|rcutils' \
    && echo "WARNING: binary links against ROS" \
    || echo "no ROS libraries linked"
