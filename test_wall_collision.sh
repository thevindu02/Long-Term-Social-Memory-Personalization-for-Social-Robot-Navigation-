#!/bin/bash
export IGN_CONFIG_PATH=/home/rosdev/gazebo_workspace/install/hunav_gazebo_fortress_wrapper/share/hunav_gazebo_fortress_wrapper/config
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && source /home/rosdev/gazebo_workspace/install/setup.bash && ros2 launch social_nav_env hunav_env_launch.py headless:=true use_sim_time:=true map:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/maps/proxy_map.yaml world:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/worlds/proxy_map.sdf initial_pose_x:=0.0 initial_pose_y:=0.0" > /tmp/headless_collision.log 2>&1 &
SIM_PID=$!
sleep 15
echo "Publishing velocity towards the wall (y-direction) for 5 seconds..."
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub /cmd_vel geometry_msgs/Twist \"{linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" -r 10" &
PUB_PID=$!
sleep 10
echo "Robot position after hitting wall:"
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /odom --once | grep -A 3 'pose:\|position:'"
kill $PUB_PID
kill $SIM_PID
echo "Done."
