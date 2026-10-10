#!/bin/bash
export IGN_CONFIG_PATH=/home/rosdev/gazebo_workspace/install/hunav_gazebo_fortress_wrapper/share/hunav_gazebo_fortress_wrapper/config
LOG_FILE="/tmp/stage_a_evidence.log"
echo "=== STAGE A EVIDENCE ===" > $LOG_FILE

# Ensure docker is running
docker start benchmark_test > /dev/null

###########################################
# 4. MAP EVIDENCE
###########################################
echo "" >> $LOG_FILE
echo "=== 4. MAP ===" >> $LOG_FILE
echo "cat /home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/maps/proxy_map.yaml:" >> $LOG_FILE
docker exec benchmark_test cat /home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/maps/proxy_map.yaml >> $LOG_FILE
echo "Image size (width x height):" >> $LOG_FILE
docker exec benchmark_test bash -c "head -n 3 /home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/maps/proxy_map.pgm" >> $LOG_FILE

echo "Checking pixel columns for bot_3 door (x=2.5 to 3.5)..." >> $LOG_FILE
# Width = 50m / 0.05 = 1000. Height = 30m / 0.05 = 600.
# Origin = [-25.0, -15.0].
# y = -2.0 is row: (15.0 - 2.0)/0.05 = 260 from bottom, so row 600-260 = 340.
# Let's extract row 340 and check cols 545 to 575.
# A PGM has header (3 lines) then binary data.
docker exec benchmark_test python3 -c '
import numpy as np
with open("/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/maps/proxy_map.pgm", "rb") as f:
    f.readline(); f.readline(); f.readline()
    data = f.read()
    img = np.frombuffer(data, dtype=np.uint8).reshape((600, 1000))
    row = 340
    print("Row 340, cols 545 to 575 (254=free, 0=wall):")
    print(img[row, 545:576])
' >> $LOG_FILE

###########################################
# 3. RTF EVIDENCE
###########################################
echo "" >> $LOG_FILE
echo "=== 3. RTF ===" >> $LOG_FILE
echo "Starting headless sim with 8 agents for RTF measurement..." >> $LOG_FILE
docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && source /home/rosdev/gazebo_workspace/install/setup.bash && ros2 launch social_nav_env hunav_env_launch.py headless:=true use_sim_time:=true map:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/maps/proxy_map.yaml world:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/worlds/proxy_map.sdf initial_pose_x:=0.0 initial_pose_y:=0.0" > /dev/null 2>&1 &
SIM_PID=$!
sleep 20
docker exec benchmark_test bash -c "ign topic -e -t /world/proxy_map/stats" > /tmp/stats.log &
STATS_PID=$!
sleep 60
docker exec benchmark_test bash -c "pkill -f 'ign topic'"
# Calculate average RTF
docker exec benchmark_test python3 -c '
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
' >> $LOG_FILE
docker exec benchmark_test bash -c "pkill -f 'ros2'; pkill -f 'ign'; pkill -f 'ruby'"
sleep 5

###########################################
# 2. SCAN CHECK
###########################################
echo "" >> $LOG_FILE
echo "=== 2. SCAN CHECK ===" >> $LOG_FILE
for pos in "-9.0 0.0 1.57" "0.0 0.0 1.57"; do
    x=$(echo $pos | cut -d' ' -f1)
    y=$(echo $pos | cut -d' ' -f2)
    yaw=$(echo $pos | cut -d' ' -f3)
    echo "Running at x=$x, y=$y, yaw=$yaw (facing +y)..." >> $LOG_FILE
    docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && source /home/rosdev/gazebo_workspace/install/setup.bash && ros2 launch social_nav_env hunav_env_launch.py headless:=true use_sim_time:=true map:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/maps/proxy_map.yaml world:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/worlds/proxy_map.sdf initial_pose_x:=$x initial_pose_y:=$y initial_pose_yaw:=$yaw" > /dev/null 2>&1 &
    SIM_PID=$!
    sleep 20
    docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /scan --once" > /tmp/scan_${x}.log
    docker exec benchmark_test python3 -c "
import yaml
import sys
try:
    with open('/tmp/scan_${x}.log', 'r') as f:
        data = yaml.safe_load(f)
        ranges = data.get('ranges', [])
        # middle index (straight ahead)
        mid = len(ranges)//2
        print(f'Range straight ahead (index {mid}): {ranges[mid]}')
        if '$x' == '-9.0':
            # Find jump for door edge
            print('Scan segment showing door edge (indices 180 to 220):')
            for i in range(180, 220):
                if abs(ranges[i] - ranges[i-1]) > 1.0:
                    print(f'  Jump at index {i}: {ranges[i-1]:.2f} -> {ranges[i]:.2f}')
                    break
except Exception as e:
    print('Error parsing scan:', e)
" >> $LOG_FILE
    docker exec benchmark_test bash -c "pkill -f 'ros2'; pkill -f 'ign'; pkill -f 'ruby'"
    sleep 5
done

###########################################
# 1. NAV2 DOOR TESTS
###########################################
echo "" >> $LOG_FILE
echo "=== 1. NAV2 DOOR TESTS ===" >> $LOG_FILE

# Arrays for tests: name, door width, start_x, start_y, goal_x, goal_y, y_min, y_max
tests=(
    "top_1 2.0m -9.0 0.0 -9.0 4.0 1.85 2.15"
    "bot_1 1.5m -9.0 0.0 -9.0 -4.0 -2.15 -1.85"
    "bot_3 1.0m 3.0 0.0 3.0 -4.0 -2.15 -1.85"
)

for test in "${tests[@]}"; do
    read name width start_x start_y goal_x goal_y y_min y_max <<< "$test"
    echo "Testing door $name ($width) - 3 runs" >> $LOG_FILE
    
    for run in 1 2 3; do
        echo "  Run $run:" >> $LOG_FILE
        # Start sim
        docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && source /home/rosdev/gazebo_workspace/install/setup.bash && ros2 launch social_nav_env hunav_env_launch.py headless:=true use_sim_time:=true map:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/maps/proxy_map.yaml world:=/home/rosdev/gazebo_workspace/install/social_nav_env/share/social_nav_env/worlds/proxy_map.sdf initial_pose_x:=$start_x initial_pose_y:=$start_y" > /dev/null 2>&1 &
        SIM_PID=$!
        sleep 20 # Wait for nav2
        
        # Log odom in background
        docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /odom" > /tmp/odom_log.txt &
        ODOM_PID=$!
        
        # Send goal using action client
        docker exec benchmark_test bash -c "source /opt/ros/humble/setup.bash && ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: $goal_x, y: $goal_y}, orientation: {w: 1.0}}}}\" --feedback" > /tmp/nav_res.log
        
        docker exec benchmark_test bash -c "pkill -f 'ros2 topic echo'"
        
        # Extract status
        grep "Goal finished with status:" /tmp/nav_res.log >> $LOG_FILE
        
        # Extract crossing positions
        docker exec benchmark_test python3 -c "
import sys
y_min = float('$y_min')
y_max = float('$y_max')
crossings = []
try:
    with open('/tmp/odom_log.txt', 'r') as f:
        x = None
        y = None
        for line in f:
            if 'x:' in line and 'position' not in line:
                val = float(line.split(':')[1].strip())
                if x is None:
                    x = val
                else:
                    pass # orientation x
            elif 'y:' in line:
                val = float(line.split(':')[1].strip())
                if y is None:
                    y = val
                else:
                    pass # orientation y
            elif 'z:' in line:
                # We have x and y for position
                if x is not None and y is not None:
                    # Check if y is within the door threshold
                    if min(y_min, y_max) <= y <= max(y_min, y_max):
                        crossings.append((x, y))
                x = None
                y = None
    if crossings:
        # Just print a few samples to show it crossed
        print(f'    Crossing poses (sample): {crossings[0]}, {crossings[len(crossings)//2]}, {crossings[-1]}')
        x_vals = [c[0] for c in crossings]
        print(f'    X bounds while crossing: min {min(x_vals):.3f}, max {max(x_vals):.3f}')
    else:
        print('    No crossing poses recorded (odom may be sparse or robot did not reach door).')
except Exception as e:
    print('    Error reading odom:', e)
" >> $LOG_FILE

        # Clean up
        docker exec benchmark_test bash -c "pkill -f 'ros2'; pkill -f 'ign'; pkill -f 'ruby'"
        sleep 5
    done
done

echo "ALL TESTS COMPLETED" >> $LOG_FILE
