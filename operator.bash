#!/bin/bash
set -e -o pipefail

if [ -z "$OPERATOR" ]; then
    echo "Error: OPERATOR environment variable is not defined."
    echo "Please set it with: export OPERATOR=your_name"
    echo "Or run with OPERATOR=your_name bash operator.bash"
    exit 1
fi

# Cleanup function to kill background processes
cleanup() {
    echo "Shutting down..."
    kill -KILL -- -"$PID_KEY" 2>/dev/null  || true
    kill -KILL -- -"$PID_JOY" 2>/dev/null || true
    wait "$PID_KEY" 2>/dev/null || true
    wait "$PID_JOY" 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM EXIT

setsid ros2 run keyboard_event keyboard --ros-args -r __ns:="/${OPERATOR}" &
PID_KEY=$!
setpgid "$PID_KEY" "$PID_KEY" 2>/dev/null || true

setsid ros2 run joy joy_node --ros-args -r __ns:="/${OPERATOR}" -p deadzone:=0.025 -p autorepeat_rate:=0.0 &
PID_JOY=$!
setpgid "$PID_JOY" "$PID_JOY" 2>/dev/null || true

ros2 run ms_operator operator \
    --ros-args \
    -p joint_speed:=0.15 \
    -p wheel_speed:=0.2 \
    -p translation_speed:=50.0 \
    -p rotation_speed:=0.15 \
    -r /keydown:="/${OPERATOR}/keydown" \
    -r /keyup:="/${OPERATOR}/keyup" \
    -r /joy:="/${OPERATOR}/joy"
