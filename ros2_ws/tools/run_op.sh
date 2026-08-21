#!/bin/bash
set -eo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
g++ -std=c++23 -O2 -I"$DIR/../src/event_system_core/src" "$DIR/op_playground.cpp" -o /tmp/op_playground
cd "$DIR"
/tmp/op_playground
