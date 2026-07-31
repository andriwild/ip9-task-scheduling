#!/usr/bin/env bash
set -euo pipefail

RADIUS="${1:?usage: ./build_tours.sh <radius>}"
cd "$(cd "$(dirname "$0")" && pwd)"

cmake -S tools/tour_gen -B tools/tour_gen/build -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build tools/tour_gen/build -j >/dev/null

exec ./tools/tour_gen/build/tour_gen "$RADIUS"
