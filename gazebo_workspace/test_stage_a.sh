#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/rosdev/gazebo_workspace/install/setup.bash

echo '--- 1. Starting headless Gazebo & Nav2 WITHOUT HuNavSim ---'
ros2 launch social_nav_env env_launch.py headless:=True > /tmp/nav_launch.log 2>&1 &
SIM_PID=$!

echo 'Waiting 20 seconds for initialization...'
sleep 20

echo '--- 2. Running LiDAR Test ---'
python3 -u /home/rosdev/gazebo_workspace/test_lidar.py

echo '--- 3. Running Nav2 Door Test ---'
python3 -u /home/rosdev/gazebo_workspace/test_nav_doors.py

echo 'Killing simulation...'
kill $SIM_PID
sleep 2
pkill -INT -f 'ign gazebo'
pkill -INT -f 'ros2'
pkill -INT -f 'component_container'
pkill -INT -f 'parameter_bridge'
sleep 5

echo '--- 4. Starting headless Gazebo & Nav2 WITH HuNavSim (RTF Test) ---'
# hunav_env_launch.py launches Gazebo, Nav2, and HuNavSim
ros2 launch social_nav_env hunav_env_launch.py headless:=True > /tmp/rtf_launch.log 2>&1 &
RTF_PID=$!

echo 'Waiting 15 seconds for initialization...'
sleep 15

echo 'Recording RTF for 60 seconds...'
# gz stats output looks like: Real time factor: 0.99
# we will capture it for 60s
timeout 60 ign topic -e -t /world/proxy_map/stats > /tmp/gz_stats.txt

echo 'Analyzing RTF...'
grep -oP 'real_time_factor:\s*\K\d+(\.\d+)?' /tmp/gz_stats.txt > /tmp/rtf_values.txt
MEAN=$(awk '{ total += $1; count++ } END { print total/count }' /tmp/rtf_values.txt)
MIN=$(awk 'NR==1{min=$1} {if($1<min) min=$1} END {print min}' /tmp/rtf_values.txt)

echo "RTF Mean: $MEAN"
echo "RTF Minimum: $MIN"

echo 'Killing simulation...'
kill $RTF_PID
sleep 2
pkill -INT -f 'ign gazebo'
pkill -INT -f 'ros2'
pkill -INT -f 'component_container'
pkill -INT -f 'parameter_bridge'
sleep 5
echo '--- Done ---'
