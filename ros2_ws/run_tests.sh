#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

echo "Building tests..."
colcon build --packages-select event_system_core --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -3

echo ""
echo "=== Running Tests ==="
echo ""

FAILED=0

for test in build/event_system_core/test_*; do
    if [ -x "$test" ]; then
        echo "--- $(basename "$test") ---"
        if ! "$test"; then
            FAILED=1
        fi
        echo ""
    fi
done

if [ $FAILED -ne 0 ]; then
    echo "SOME TESTS FAILED"
    exit 1
else
    echo "ALL TESTS PASSED"
fi
