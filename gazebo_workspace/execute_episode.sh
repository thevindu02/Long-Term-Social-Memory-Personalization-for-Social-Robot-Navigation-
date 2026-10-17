#!/bin/bash
set -e

SCENARIO=${1:-corridor}
MODE=${2:-baseline}
RUN_ID=${3:-run_1}
SEED=${4:-42}
JITTER=${5:-0.0}
TIMEOUT=${6:-135}

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
echo "=== 2. Launching Gazebo & Nav2 (Scenario: ${SCENARIO}, Config: ${CONFIG_FILE}) Headless ==="
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{time}] [{name}] [{severity}]: {message}"
ros2 launch social_nav_env hunav_env_launch.py headless:=True configuration_file:=${CONFIG_FILE} > /tmp/sim_launch.log 2>&1 &
SIM_PID=$!

cleanup() {
    echo "=== Post-Run Cleanup ==="
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

echo "=== 3. Executing run_scenario_episode.py ==="
python3 -u /home/rosdev/gazebo_workspace/run_scenario_episode.py \
    --scenario "${SCENARIO}" \
    --mode "${MODE}" \
    --run-id "${RUN_ID}" \
    --seed "${SEED}" \
    --jitter "${JITTER}" \
    --timeout "${TIMEOUT}" \
    --output-dir "/home/rosdev/gazebo_workspace/evaluation_results"
