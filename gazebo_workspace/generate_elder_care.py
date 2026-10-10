#!/usr/bin/env python3
import struct
import math
import os
import yaml

RESOLUTION = 0.05
WIDTH_M = 50.0
HEIGHT_M = 30.0
W = int(WIDTH_M / RESOLUTION)
H = int(HEIGHT_M / RESOLUTION)

# 254 = free space, 0 = occupied, 205 = unknown
grid = [[254 for _ in range(W)] for _ in range(H)]

# We will collect walls for the SDF
walls = []

def draw_wall(cx, cy, sx, sy, name="wall"):
    px = W/2 + (cx / RESOLUTION)
    py = H/2 - (cy / RESOLUTION)
    pw = sx / RESOLUTION
    ph = sy / RESOLUTION
    
    x1 = int(round(px - pw/2))
    x2 = int(round(px + pw/2))
    y1 = int(round(py - ph/2))
    y2 = int(round(py + ph/2))
    
    x1 = max(0, min(W-1, x1))
    x2 = max(0, min(W-1, x2))
    y1 = max(0, min(H-1, y1))
    y2 = max(0, min(H-1, y2))
    
    for y in range(y1, y2):
        for x in range(x1, x2):
            grid[y][x] = 0
            
    walls.append((cx, cy, sx, sy, name))

W_THICK = 0.3


# Corridor horizontal walls (with properly centered gaps for doors)
doors = [
    (-9.0, 2.0, True),
    (-3.0, 1.5, True),
    ( 3.0, 1.5, True),
    ( 9.0, 1.5, True),
    (-9.0, 1.5, False),
    (-3.0, 1.5, False),
    ( 3.0, 1.0, False),
    ( 9.0, 1.5, False),
]

rooms_x = [(-12.0, -6.0), (-6.0, 0.0), (0.0, 6.0), (6.0, 12.0)]
for room_idx, (rx_start, rx_end) in enumerate(rooms_x):
    cx = rx_start + 3.0
    
    # Top wall
    gap_w = [d[1] for d in doors if d[0] == cx and d[2]][0]
    seg1_start = rx_start
    seg1_end = cx - gap_w/2
    seg1_len = seg1_end - seg1_start
    if seg1_len > 0:
        draw_wall(seg1_start + seg1_len/2, 2.0, seg1_len, W_THICK, f"corr_t_{room_idx}_1")
    seg2_start = cx + gap_w/2
    seg2_end = rx_end
    seg2_len = seg2_end - seg2_start
    if seg2_len > 0:
        draw_wall(seg2_start + seg2_len/2, 2.0, seg2_len, W_THICK, f"corr_t_{room_idx}_2")
    print(f"Door top_{room_idx+1}: gap {seg1_end} to {seg2_start}, width {gap_w}")
        
    # Bottom wall
    gap_w_bot = [d[1] for d in doors if d[0] == cx and not d[2]][0]
    seg1_start_b = rx_start
    seg1_end_b = cx - gap_w_bot/2
    seg1_len_b = seg1_end_b - seg1_start_b
    if seg1_len_b > 0:
        draw_wall(seg1_start_b + seg1_len_b/2, -2.0, seg1_len_b, W_THICK, f"corr_b_{room_idx}_1")
    seg2_start_b = cx + gap_w_bot/2
    seg2_end_b = rx_end
    seg2_len_b = seg2_end_b - seg2_start_b
    if seg2_len_b > 0:
        draw_wall(seg2_start_b + seg2_len_b/2, -2.0, seg2_len_b, W_THICK, f"corr_b_{room_idx}_2")
    print(f"Door bot_{room_idx+1}: gap {seg1_end_b} to {seg2_start_b}, width {gap_w_bot}")

# Vertical walls separating rooms
for x in [-12.0, -6.0, 0.0, 6.0, 12.0]:
    draw_wall(x, 5.0, W_THICK, 6.0, f"room_v_t_{x}")
    draw_wall(x, -5.0, W_THICK, 6.0, f"room_v_b_{x}")

# Outer boundaries
draw_wall(0.0, 8.0, 24.0 + W_THICK, W_THICK, "outer_top")
draw_wall(0.0, -8.0, 24.0 + W_THICK, W_THICK, "outer_bot")
draw_wall(-12.0, 0.0, W_THICK, 4.0, "outer_left_corr")
draw_wall(12.0, 0.0, W_THICK, 4.0, "outer_right_corr")

# Add a common area at the right (x from 12 to 20, y from -4 to 4)
draw_wall(16.0, 4.0, 8.0, W_THICK, "common_top")
draw_wall(16.0, -4.0, 8.0, W_THICK, "common_bot")
draw_wall(20.0, 0.0, W_THICK, 8.0, "common_right")

# Floodfill for map
def flood_fill(sx, sy, target, replace):
    if sx < 0 or sx >= W or sy < 0 or sy >= H: return
    if grid[sy][sx] != target: return
    stack = [(sx, sy)]
    while stack:
        x, y = stack.pop()
        if grid[y][x] == target:
            grid[y][x] = replace
            if x > 0: stack.append((x-1, y))
            if x < W-1: stack.append((x+1, y))
            if y > 0: stack.append((x, y-1))
            if y < H-1: stack.append((x, y+1))

flood_fill(0, 0, 254, 205) # Assume top-left is unknown

# Generate Map PGM/YAML
out_dir = "src/social_nav_env/maps"
os.makedirs(out_dir, exist_ok=True)
with open(os.path.join(out_dir, "proxy_map.pgm"), 'wb') as f:
    f.write(f"P5\n{W} {H}\n255\n".encode('ascii'))
    for row in grid:
        f.write(bytearray(row))

yaml_data = f"""image: proxy_map.pgm
resolution: {RESOLUTION}
origin: [{-WIDTH_M/2}, {-HEIGHT_M/2}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
with open(os.path.join(out_dir, "proxy_map.yaml"), 'w') as f:
    f.write(yaml_data)

# Generate SDF
sdf_content = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="proxy_map">
    <physics name="default" type="ode">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="libignition-gazebo-contact-system.so" name="ignition::gazebo::systems::Contact"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 -0.005 0 0 0</pose>
      <link name="link">
        <collision name="floor_col">
          <geometry><box><size>100 100 0.01</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="vis">
          <geometry><box><size>100 100 0.01</size></box></geometry>
          <material>
            <ambient>0.6 0.6 0.5 1</ambient>
            <diffuse>0.6 0.6 0.5 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""

# Add walls
for w in walls:
    cx, cy, sx, sy, name = w
    sdf_content += f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} 1.0 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>{sx} {sy} 2.0</size></box></geometry></collision>
        <visual name="vis">
          <geometry><box><size>{sx} {sy} 2.0</size></box></geometry>
          <material>
            <ambient>0.9 0.85 0.8 1</ambient>
            <diffuse>0.9 0.85 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

# Add Fuel Models (Furniture)
# Beds in rooms
for rx in [-9.0, -3.0, 3.0, 9.0]:
    # Top rooms
    sdf_content += f"""
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/TrolleyBed</uri>
      <name>bed_t_{rx}</name>
      <pose>{rx} 6.5 0 0 0 0</pose>
    </include>
"""
    # Bottom rooms
    sdf_content += f"""
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/TrolleyBed</uri>
      <name>bed_b_{rx}</name>
      <pose>{rx} -6.5 0 0 0 3.1415</pose>
    </include>
"""

# Common area furniture
sdf_content += """
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sofa</uri>
      <name>sofa1</name>
      <pose>18.0 2.0 0 0 0 -1.57</pose>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/VisitorChair</uri>
      <name>chair1</name>
      <pose>16.0 2.0 0 0 0 1.57</pose>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/PatientWheelChair</uri>
      <name>wheelchair1</name>
      <pose>14.0 -2.0 0 0 0 3.14</pose>
    </include>
"""

sdf_content += """
    <model name="simple_robot">
      <pose>0 0 0.15 0 0 0</pose>
      <link name="base_link">
        <inertial><mass>10.0</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>
        <visual name="visual">
          <geometry><cylinder><radius>0.2</radius><length>0.2</length></cylinder></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient><diffuse>0.2 0.2 0.8 1</diffuse></material>
        </visual>
        <collision name="collision">
          <geometry><cylinder><radius>0.2</radius><length>0.2</length></cylinder></geometry>
        </collision>
      </link>
      <link name="lidar_link">
        <pose>0 0 0.15 0 0 0</pose>
        <visual name="vis">
          <geometry><cylinder><radius>0.05</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>0.2 0.8 0.2 1</ambient></material>
        </visual>
        <sensor name="lidar" type="gpu_lidar">
          <always_on>true</always_on>
          <update_rate>10</update_rate>
          <topic>/scan</topic>
          <ray>
            <scan><horizontal><samples>360</samples><resolution>1</resolution><min_angle>-3.1415</min_angle><max_angle>3.1415</max_angle></horizontal></scan>
            <range><min>0.12</min><max>20.0</max><resolution>0.01</resolution></range>
          </ray>
        </sensor>
      </link>
      <joint name="lidar_joint" type="fixed"><parent>base_link</parent><child>lidar_link</child></joint>
      <plugin filename="libignition-gazebo-velocity-control-system.so" name="ignition::gazebo::systems::VelocityControl">
        <topic>/cmd_vel</topic>
      </plugin>
      <plugin filename="libignition-gazebo-odometry-publisher-system.so" name="ignition::gazebo::systems::OdometryPublisher">
        <odom_frame>odom</odom_frame>
        <robot_base_frame>base_link</robot_base_frame>
        <odom_topic>/odom</odom_topic>
        <tf_topic>/tf</tf_topic>
        <odom_publish_frequency>20</odom_publish_frequency>
      </plugin>
    </model>
  </world>
</sdf>
"""

with open("src/social_nav_env/worlds/proxy_map.sdf", "w") as f:
    f.write(sdf_content)

# Generate Agents YAML
agents_yaml = {
    "hunav_loader": {
        "ros__parameters": {
            "yaml_base_name": "proxy_agents",
            "simulator": "Gazebo Fortress",
            "map": "proxy_map",
            "publish_people": True,
            "global_goals": {
                1: {"x": -10.0, "y": 0.0},
                2: {"x": 10.0, "y": 0.0},
                3: {"x": -9.0, "y": 5.0},
                4: {"x": 9.0, "y": 5.0},
                5: {"x": -9.0, "y": -5.0},
                6: {"x": 9.0, "y": -5.0},
                7: {"x": 18.0, "y": 2.0},
                8: {"x": 18.0, "y": -2.0}
            },
            "agents": [f"agent{i}" for i in range(1, 9)]
        }
    }
}

for i in range(1, 9):
    agent_data = {
        "id": i,
        "group_id": -1,
        "skin": i % 4,
        "max_vel": 1.0,
        "radius": 0.4,
        "goal_radius": 0.5,
        "cyclic_goals": True,
        "init_pose": {
            "x": (i - 4.5) * 3.0,
            "y": 0.0 if i % 2 == 0 else (2.0 if i % 4 == 1 else -2.0),
            "z": 1.25,
            "h": 0.0
        },
        "behavior": {
            "type": "Regular",
            "configuration": 0,
            "goal_force_factor": 2.0,
            "obstacle_force_factor": 10.0,
            "social_force_factor": 5.0,
            "other_force_factor": 20.0
        },
        "goals": [(i % 8) + 1, ((i+2) % 8) + 1, ((i+4) % 8) + 1]
    }
    agents_yaml["hunav_loader"]["ros__parameters"][f"agent{i}"] = agent_data

with open("src/hunav_gazebo_fortress_wrapper/scenarios/proxy_agents.yaml", "w") as f:
    yaml.dump(agents_yaml, f, sort_keys=False)

print("Generated new proxy_map.sdf, proxy_map.pgm, proxy_map.yaml, and proxy_agents.yaml!")
