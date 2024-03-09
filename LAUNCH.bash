#!/bin/bash

# Function to check if any process is still running
is_running() {
    for pid in "$@"; do
        if ps -p $pid > /dev/null; then
            return 0  # Process is still running
        fi
    done
    return 1  # All processes have finished
}

# Function to cleanup and kill background processes
cleanup() {
    echo "Caught Ctrl-C. Cleaning up and killing background processes."
    pstree $$
        
    # Iterate over main processes and their children to collect all PIDs
    all_pids=("${pids[@]}")
    for pid in "${pids[@]}"; do
        child_pids=$(pgrep -P $pid)
        all_pids+=($child_pids)
    done

    # Remove duplicate PIDs (if any)
    all_pids=($(echo "${all_pids[@]}" | tr ' ' '\n' | sort -u | tr '\n' ' '))
    echo "terminating ${all_pids[@]}"

    for pid in "${all_pids[@]}"; do
        pkill -SIGINT -P $pid
        if ps -p $pid > /dev/null; then
            kill -SIGINT $pid
            echo "Sending SIGINT timeout to process $pid"
        fi
    done

    # Wait until all processes have finished or timeout occurs
    start_time=$(date +%s)
    timeout_duration=20
    while is_running "${all_pids[@]}"; do
        current_time=$(date +%s)
        elapsed_time=$((current_time - start_time))
        if [ $elapsed_time -ge $timeout_duration ]; then
            echo "SIGINT timeout reached"
            break
        fi
        sleep 1
    done

    for pid in "${all_pids[@]}"; do
        pkill -SIGTERM -P $pid
        if ps -p $pid > /dev/null; then
            kill -SIGTERM $pid
            echo "Sending SIGTERM timeout to process $pid"
        fi
    done

    # Wait until all processes have finished or timeout occurs
    start_time=$(date +%s)
    timeout_duration=10
    while is_running "${all_pids[@]}"; do
        current_time=$(date +%s)
        elapsed_time=$((current_time - start_time))
        if [ $elapsed_time -ge $timeout_duration ]; then
            echo "SIGTERM timeout reached"
            break
        fi
        sleep 1
    done

    for pid in "${all_pids[@]}"; do
        pkill -SIGKILL -P $pid
        if ps -p $pid > /dev/null; then
            kill -SIGKILL $pid
            echo "Sending SIGKILL timeout to process $pid"
        fi
    done

    wait "${all_pids[@]}"
    echo "All process terminated"
}

# Traps termination signal for proper handling
trap cleanup SIGINT
trap cleanup SIGTERM
trap cleanup SIGKILL

cd "${ROS2_MOONBOT_WS}" || exit
. "${ROS2_INSTALL_PATH}"/setup.bash
export ROS_DOMAIN_ID=58
# rm -rf install
# rm -rf build
. install/setup.bash
colcon build --symlink-install || exit
. install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1

# rqt || exit &
. 04BRL_easy_control.bash || exit &
ros2 run pcl_reader pointcloud_read_pub || exit &
. BL_rviz.bash || exit &

# Store PIDs of background processes
pids=($(jobs -p))

# Wait for all background processes to finish
wait
