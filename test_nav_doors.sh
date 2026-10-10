#!/bin/bash
export IGN_CONFIG_PATH=/home/rosdev/gazebo_workspace/install/hunav_gazebo_fortress_wrapper/share/hunav_gazebo_fortress_wrapper/config
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && source /home/rosdev/gazebo_workspace/install/setup.bash && ros2 launch social_nav_env hunav_env_launch.py headless:=true use_sim_time:=true map:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/maps/proxy_map.yaml world:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/worlds/proxy_map.sdf initial_pose_x:=0.0 initial_pose_y:=0.0" > /tmp/headless_nav_doors.log 2>&1 &
SIM_PID=$!
echo "Starting headless simulation to test doors..."
sleep 15
echo "Sending navigation goal 1 (x=3.0, y=-4.0 - Through 1.0m door bot_3)..."
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \"{header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: -4.0, z: 0.0}, orientation: {w: 1.0}}}\""
sleep 20
echo "Sending navigation goal 2 (x=-9.0, y=4.0 - Through 2.0m door top_1)..."
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \"{header: {frame_id: 'map'}, pose: {position: {x: -9.0, y: 4.0, z: 0.0}, orientation: {w: 1.0}}}\""
sleep 30
echo "Sending navigation goal 3 (x=-3.0, y=-4.0 - Through 1.5m door bot_2)..."
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \"{header: {frame_id: 'map'}, pose: {position: {x: -3.0, y: -4.0, z: 0.0}, orientation: {w: 1.0}}}\""
sleep 20
kill $SIM_PID
echo "Done."
