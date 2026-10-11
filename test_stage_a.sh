#!/bin/bash
export IGN_CONFIG_PATH=/home/rosdev/gazebo_workspace/install/hunav_gazebo_fortress_wrapper/share/hunav_gazebo_fortress_wrapper/config

echo "Killing any existing ROS/Gazebo processes..."
docker exec benchmark_test bash -c "pkill -f 'ros2'; pkill -f 'ign'; pkill -f 'ruby'"

echo "Starting headless simulation for Stage A verification..."
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && source /home/rosdev/gazebo_workspace/install/setup.bash && ros2 launch social_nav_env hunav_env_launch.py headless:=true use_sim_time:=true map:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/maps/proxy_map.yaml world:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/worlds/proxy_map.sdf initial_pose_x:=0.0 initial_pose_y:=0.0" > /tmp/headless_stage_a.log 2>&1 &
SIM_PID=$!
sleep 20

echo "1. capturing real /scan..."
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /scan --once" > /tmp/real_scan.log

echo "2. Raw velocity-override wall collision test (Publishing vy=1.0 for 5s)..."
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub /cmd_vel geometry_msgs/Twist \"{linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" -r 10" &
PUB_PID=$!
sleep 5
echo "Robot position after hitting wall:"
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /odom --once | grep -A 3 'pose:\|position:'" > /tmp/wall_collision.log
docker exec benchmark_test bash -c "pkill -f 'ros2 topic pub'"

echo "3. Nav2 test through 1.0m door bot_3 (Goal 3.0, -4.0)..."
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \"{header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: -4.0, z: 0.0}, orientation: {w: 1.0}}}\""
sleep 20

echo "Robot position after navigating to 1.0m door bot_3:"
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /odom --once | grep -A 3 'pose:\|position:'" > /tmp/nav2_door.log

docker exec benchmark_test bash -c "pkill -f 'ros2'; pkill -f 'ign'; pkill -f 'ruby'"
echo "Done."
