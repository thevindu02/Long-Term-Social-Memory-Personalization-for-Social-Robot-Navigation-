#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/rosdev/gazebo_workspace/install/setup.bash

echo "=== 1. Cleaning up previous processes and recordings ==="
pkill -9 -f 'ros2' || true
pkill -9 -f 'ign' || true
pkill -9 -f 'ruby' || true
pkill -9 -f 'parameter_bridge' || true
pkill -9 -f 'component_container' || true
pkill -9 -f 'hunav' || true
rm -rf /home/rosdev/gazebo_workspace/sim_record
sleep 2

echo "=== 2. Launching Gazebo & Nav2 with Corridor Scenario Headless ==="
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{time}] [{name}] [{severity}]: {message}"
ros2 launch social_nav_env hunav_env_launch.py headless:=True configuration_file:=scenario_corridor.yaml > /tmp/gt_launch.log 2>&1 &
SIM_PID=$!

cleanup() {
    echo "=== Cleaning up simulation ==="
    kill -9 $SIM_PID 2>/dev/null || true
    pkill -9 -f 'ros2' || true
    pkill -9 -f 'ign' || true
    pkill -9 -f 'ruby' || true
    pkill -9 -f 'parameter_bridge' || true
    pkill -9 -f 'component_container' || true
    pkill -9 -f 'hunav' || true
    rm -rf /home/rosdev/gazebo_workspace/sim_record
}
trap cleanup EXIT

echo "=== 3. Running check_ground_truth.py ==="
python3 -u /home/rosdev/gazebo_workspace/check_ground_truth.py 2>&1 | tee /tmp/gt_stdout.log
