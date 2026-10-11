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
sleep 3

echo "=== 2. Starting background memory logger (every 5s to mem_rtf.log) ==="
rm -f /home/rosdev/gazebo_workspace/mem_rtf.log
(
  while true; do
    echo "$(date +%s) $(free -m | awk '/Mem:/ {print $3}')" >> /home/rosdev/gazebo_workspace/mem_rtf.log
    sleep 5
  done
) &
MEM_PID=$!

echo "=== 3. Launching Gazebo & Nav2 WITH HuNavSim (8 agents) Headless ==="
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{time}] [{name}] [{severity}]: {message}"
ros2 launch social_nav_env hunav_env_launch.py headless:=True > /tmp/hunav_launch.log 2>&1 &
SIM_PID=$!

cleanup() {
    echo "=== Cleaning up simulation and memory logger ==="
    kill -9 $MEM_PID 2>/dev/null || true
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

echo "=== 4. Running test_rtf.py ==="
python3 -u /home/rosdev/gazebo_workspace/test_rtf.py 2>&1 | tee /tmp/rtf_stdout.log

echo "=== 5. Memory Statistics ==="
PEAK_MEM=$(sort -k2 -n /home/rosdev/gazebo_workspace/mem_rtf.log | tail -n 1 | awk '{print $2}')
echo "Peak memory used: ${PEAK_MEM} MB"
