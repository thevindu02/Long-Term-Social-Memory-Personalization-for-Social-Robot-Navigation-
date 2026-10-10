#!/bin/bash
OUT="/tmp/stage_a_final.log"
echo "=== STAGE A EVIDENCE ===" > $OUT

# Helper to run docker exec safely
dexec() {
    docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && source /home/rosdev/gazebo_workspace/install/setup.bash && $*"
}

# 1. RTF Test
echo "=== 3. RTF ===" >> $OUT
echo "Starting sim for RTF..." >> $OUT
dexec "ros2 launch social_nav_env hunav_env_launch.py headless:=true use_sim_time:=true map:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/maps/proxy_map.yaml world:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/worlds/proxy_map.sdf initial_pose_x:=0.0 initial_pose_y:=0.0" > /dev/null 2>&1 &
sleep 20
docker exec benchmark_test bash -c "ign topic -e -t /world/proxy_map/stats" > /tmp/stats.log &
sleep 60
docker exec benchmark_test pkill -f "ign topic"
python3 -c '
import re
rtfs = []
with open("/tmp/stats.log", "r") as f:
    for line in f:
        if "real_time_factor:" in line:
            rtfs.append(float(line.split(":")[1].strip()))
if rtfs:
    print(f"Average RTF over 60s: {sum(rtfs)/len(rtfs):.4f}")
else:
    print("Could not parse RTF")
' >> $OUT
docker exec benchmark_test pkill -f 'ros2|ign|ruby'
sleep 5

# 2. SCAN Tests
echo "=== 2. SCAN CHECK ===" >> $OUT
for pos in "-9.0 0.0 1.57" "0.0 0.0 1.57"; do
    read x y yaw <<< "$pos"
    echo "Running at x=$x, y=$y, yaw=$yaw (facing +y)..." >> $OUT
    dexec "ros2 launch social_nav_env hunav_env_launch.py headless:=true use_sim_time:=true map:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/maps/proxy_map.yaml world:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/worlds/proxy_map.sdf initial_pose_x:=$x initial_pose_y:=$y initial_pose_yaw:=$yaw" > /dev/null 2>&1 &
    sleep 20
    dexec "ros2 topic echo /scan --once" > /tmp/scan_${x}.yaml
    python3 -c "
import yaml
try:
    with open('/tmp/scan_${x}.yaml', 'r') as f:
        data = yaml.safe_load(f)
        ranges = data.get('ranges', [])
        mid = len(ranges)//2
        print(f'Range straight ahead (index {mid}): {ranges[mid]:.4f}')
        if '$x' == '-9.0':
            print('Scan segment showing door edge (indices 180 to 220):')
            for i in range(180, 220):
                if abs(ranges[i] - ranges[i-1]) > 1.0:
                    print(f'  Jump at index {i}: {ranges[i-1]:.2f} -> {ranges[i]:.2f}')
                    break
except Exception as e:
    print('Error:', e)
" >> $OUT
    docker exec benchmark_test pkill -f 'ros2|ign|ruby'
    sleep 5
done

# 3. NAV2 DOOR TESTS
echo "=== 1. NAV2 DOOR TESTS ===" >> $OUT
tests=(
    "top_1 2.0m -9.0 0.0 -9.0 4.0 1.85 2.15"
    "bot_1 1.5m -9.0 0.0 -9.0 -4.0 -2.15 -1.85"
    "bot_3 1.0m 3.0 0.0 3.0 -4.0 -2.15 -1.85"
)
for test in "${tests[@]}"; do
    read name width start_x start_y goal_x goal_y y_min y_max <<< "$test"
    echo "Testing door $name ($width) - 3 runs" >> $OUT
    
    for run in 1 2 3; do
        echo "  Run $run:" >> $OUT
        dexec "ros2 launch social_nav_env hunav_env_launch.py headless:=true use_sim_time:=true map:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/maps/proxy_map.yaml world:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/worlds/proxy_map.sdf initial_pose_x:=$start_x initial_pose_y:=$start_y" > /dev/null 2>&1 &
        sleep 20
        dexec "ros2 topic echo /odom" > /tmp/odom_log.txt &
        
        # Action client WITHOUT feedback
        dexec "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: $goal_x, y: $goal_y}, orientation: {w: 1.0}}}}\"" > /tmp/nav_res.log
        
        docker exec benchmark_test pkill -f 'ros2 topic echo'
        
        grep "Goal finished with status:" /tmp/nav_res.log >> $OUT
        
        python3 -c "
y_min = float('$y_min')
y_max = float('$y_max')
crossings = []
try:
    with open('/tmp/odom_log.txt', 'r') as f:
        x = None; y = None
        for line in f:
            if 'x:' in line and 'position' not in line:
                if x is None: x = float(line.split(':')[1].strip())
            elif 'y:' in line:
                if y is None: y = float(line.split(':')[1].strip())
            elif 'z:' in line:
                if x is not None and y is not None:
                    if min(y_min, y_max) <= y <= max(y_min, y_max):
                        crossings.append((x, y))
                x = None; y = None
    if crossings:
        print(f'    Crossing poses (sample): {crossings[0]}, {crossings[len(crossings)//2]}, {crossings[-1]}')
        x_vals = [c[0] for c in crossings]
        print(f'    X bounds while crossing: min {min(x_vals):.3f}, max {max(x_vals):.3f}')
    else:
        print('    No crossing poses recorded.')
except Exception as e:
    pass
" >> $OUT
        docker exec benchmark_test pkill -f 'ros2|ign|ruby'
        sleep 5
    done
done

echo "ALL DONE" >> $OUT
