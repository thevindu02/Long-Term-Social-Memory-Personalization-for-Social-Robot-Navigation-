#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/rosdev/gazebo_workspace/install/setup.bash

echo '--- 1. Starting headless Gazebo & Nav2 WITHOUT HuNavSim ---'
ros2 launch social_nav_env env_launch.py headless:=True > /tmp/nav_launch.log 2>&1 &
SIM_PID=$!

echo 'Waiting for Nav2 initialization (up to 120s)...'
python3 -c "
import rclpy, sys, time
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
rclpy.init()
n = Node('waiter')
c = n.create_client(GetState, '/bt_navigator/get_state')
start = time.time()
while not c.wait_for_service(timeout_sec=1.0):
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.time() - start > 120: sys.exit(1)
req = GetState.Request()
while True:
    f = c.call_async(req)
    rclpy.spin_until_future_complete(n, f)
    if f.result().current_state.label == 'active': break
    rclpy.spin_once(n, timeout_sec=1.0)
    if time.time() - start > 120: sys.exit(1)
"
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
ros2 launch social_nav_env hunav_env_launch.py headless:=True > /tmp/rtf_launch.log 2>&1 &
RTF_PID=$!

python3 -u /home/rosdev/gazebo_workspace/test_rtf.py

echo 'Killing simulation...'
kill $RTF_PID 2>/dev/null || true
sleep 2
pkill -9 -f 'ign' || true
pkill -9 -f 'ros2' || true
pkill -9 -f 'component_container' || true
pkill -9 -f 'parameter_bridge' || true
pkill -9 -f 'hunav' || true
sleep 3
echo '--- Done ---'
