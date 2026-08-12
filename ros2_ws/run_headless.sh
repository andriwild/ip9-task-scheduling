#!/usr/bin/env bash
# Runs the ROS-free simulator. Builds it first if it is missing.
#
#   ./run_headless.sh                                  base config
#   ./run_headless.sh overrides/30_day.json            with an override
#   ./run_headless.sh overrides/30_day.json results/x  and a fixed output dir
#
# LOG_LEVEL=DEBUG and BASE_CONFIG=... are picked up from the environment.
set -euo pipefail

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN="$WS_DIR/build-headless/des_headless"

[ -x "$BIN" ] || "$WS_DIR/build_headless.sh"

ARGS=(--mode headless --log-level "des:=${LOG_LEVEL:-INFO}")
[ -n "${1:-}" ] && ARGS+=(--config "$1")
[ -n "${2:-}" ] && ARGS+=(--out-dir "$2")
[ -n "${BASE_CONFIG:-}" ] && ARGS+=(--base-config "$BASE_CONFIG")

cd "$WS_DIR"
exec "$BIN" "${ARGS[@]}"
