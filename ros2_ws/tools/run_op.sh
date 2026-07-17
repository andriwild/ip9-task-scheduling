#!/bin/bash
set -eo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
g++ -std=c++20 -O2 "$DIR/op_playground.cpp" -o /tmp/op_playground
/tmp/op_playground
