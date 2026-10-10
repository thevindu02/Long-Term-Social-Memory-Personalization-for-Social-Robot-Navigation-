#!/bin/bash

# Clear previous recordings
docker exec benchmark_test bash -c "rm -rf /home/rosdev/gazebo_workspace/sim_record"

echo "Starting headless simulation and recording..."
# Launch headless simulation in the background
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch social_nav_env hunav_env_launch.py headless:=True" > /tmp/headless_run.log 2>&1 &
SIM_PID=$!

echo "Waiting for Nav2 and Simulator to initialize (15 seconds)..."
sleep 15

echo "Sending navigation goal 1 (x=-9.0, y=0.0 - Down the corridor)..."
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && python3 /home/rosdev/gazebo_workspace/send_robot_goal.py -9.0 0.0"

echo "Letting the simulation run for 60 seconds..."
sleep 60

echo "Sending navigation goal 2 (x=-9.0, y=4.5 - Enter a patient room)..."
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && python3 /home/rosdev/gazebo_workspace/send_robot_goal.py -9.0 4.5"

echo "Letting the simulation run for 60 seconds..."
sleep 60

echo "Sending navigation goal 3 (x=16.0, y=0.0 - Go to the common area)..."
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && python3 /home/rosdev/gazebo_workspace/send_robot_goal.py 16.0 0.0"

echo "Letting the simulation run for 60 seconds..."
sleep 60

echo "Sending navigation goal 4 (x=0.0, y=0.0 - Return to starting position)..."
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && python3 /home/rosdev/gazebo_workspace/send_robot_goal.py 0.0 0.0"

echo "Letting the simulation run for 60 seconds..."
sleep 60
echo "Stopping the simulation..."
# Kill the simulation gracefully to save the recording
docker exec benchmark_test pkill -INT -f "ign gazebo"
sleep 2
docker exec benchmark_test pkill -INT -f "ros2"

echo "Done! You can now playback the recording in the GUI."
