#!/bin/bash
set -e

SCENARIO="corridor"
MODE="robot_present"
RUN_ID="rec_test_corridor"
SEED="42"
JITTER="0.0"
TIMEOUT="135"

source /opt/ros/humble/setup.bash
source /home/rosdev/gazebo_workspace/install/setup.bash

echo "=== 1. Pre-Run Process Cleanup ==="
pkill -9 -f 'ros2' || true
pkill -9 -f 'ign' || true
pkill -9 -f 'ruby' || true
pkill -9 -f 'parameter_bridge' || true
pkill -9 -f 'component_container' || true
pkill -9 -f 'hunav' || true
rm -rf /home/rosdev/gazebo_workspace/sim_record
sleep 2

CONFIG_FILE="scenario_${SCENARIO}.yaml"
echo "=== 2. Launching Gazebo & Nav2 Headless with Fortress Recording ==="
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{time}] [{name}] [{severity}]: {message}"
ros2 launch social_nav_env hunav_env_launch.py headless:=True configuration_file:=${CONFIG_FILE} > /tmp/sim_launch_rec.log 2>&1 &
SIM_PID=$!

cleanup() {
    echo "=== Post-Run Cleanup (Preserving sim_record) ==="
    kill -9 $SIM_PID 2>/dev/null || true
    pkill -9 -f 'ros2' || true
    pkill -9 -f 'ign' || true
    pkill -9 -f 'ruby' || true
    pkill -9 -f 'parameter_bridge' || true
    pkill -9 -f 'component_container' || true
    pkill -9 -f 'hunav' || true
}
trap cleanup EXIT

echo "=== 3. Executing run_scenario_episode.py for ${RUN_ID} ==="
python3 -u /home/rosdev/gazebo_workspace/run_scenario_episode.py \
    --scenario "${SCENARIO}" \
    --mode "${MODE}" \
    --run-id "${RUN_ID}" \
    --seed "${SEED}" \
    --jitter "${JITTER}" \
    --timeout "${TIMEOUT}" \
    --output-dir "/home/rosdev/gazebo_workspace/evaluation_results"

echo "=== 4. Verifying Fortress Recorded Log Directory ==="
ls -lh /home/rosdev/gazebo_workspace/sim_record || true
du -sh /home/rosdev/gazebo_workspace/sim_record || true
