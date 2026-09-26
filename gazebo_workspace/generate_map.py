#!/usr/bin/env python3
"""
Generates proxy_map.sdf with:
- FULL collision geometry restored on all walls (needed for LiDAR raycasting)
- Wall bottoms at z=0.01 (NOT touching floor at z=0) -> no coplanar contact
- ODE physics with relaxed constraints (the one mitigation we keep)
- Frictionless floor (mu=0) as safety net
- 2D LiDAR sensor (gpu_lidar plugin) for validation
"""

WALL_COLOR  = "0.88 0.88 0.88 1"
FLOOR_COLOR = "0.60 0.72 0.82 1"
H   = 2.0   # wall height
W   = 0.3   # wall thickness
ZC  = 1.01  # wall centre Z => bottom face at z=0.01m, NOT touching floor at z=0

def wall(name, cx, cy, sx, sy):
    """Wall with BOTH visual AND collision geometry."""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {ZC} 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>{sx} {sy} {H}</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>{sx} {sy} {H}</size></box></geometry>
          <material>
            <ambient>{WALL_COLOR}</ambient>
            <diffuse>{WALL_COLOR}</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>"""

walls = [
    # Hub N wall (gap in centre for N corridor)
    wall("hub_n_left",  -3.5,  5.0,  3.0, W),
    wall("hub_n_right",  3.5,  5.0,  3.0, W),
    # Hub S wall
    wall("hub_s_left",  -3.5, -5.0,  3.0, W),
    wall("hub_s_right",  3.5, -5.0,  3.0, W),
    # Hub E wall (gap for E corridor)
    wall("hub_e_top",    5.0,  3.5,  W,   3.0),
    wall("hub_e_bot",    5.0, -3.5,  W,   3.0),
    # Hub W wall (gap for W corridor)
    wall("hub_w_top",   -5.0,  3.5,  W,   3.0),
    wall("hub_w_bot",   -5.0, -3.5,  W,   3.0),
    # North corridor
    wall("corr_n_left",  -2.0, 10.0, W,  10.0),
    wall("corr_n_right",  2.0, 10.0, W,  10.0),
    wall("corr_n_end",    0.0, 15.0, 4.0, W),
    # South corridor
    wall("corr_s_left",  -2.0,-10.0, W,  10.0),
    wall("corr_s_right",  2.0,-10.0, W,  10.0),
    wall("corr_s_end",    0.0,-15.0, 4.0, W),
    # East corridor (door gap x=7..9 on top wall)
    wall("corr_e_bot",   10.0, -2.0, 10.0, W),
    wall("corr_e_top_1",  6.0,  2.0,  2.0, W),
    wall("corr_e_top_2", 12.0,  2.0,  6.0, W),
    wall("corr_e_end",   15.0,  0.0,  W,   4.0),
    # West corridor
    wall("corr_w_top",  -10.0,  2.0, 10.0, W),
    wall("corr_w_bot",  -10.0, -2.0, 10.0, W),
    wall("corr_w_end",  -15.0,  0.0,  W,   4.0),
    # NE Room (x=7..14, y=2..10)
    wall("room_w",   7.0,  6.0, W,   8.0),
    wall("room_e",  14.0,  6.0, W,   8.0),
    wall("room_n",  10.5, 10.0, 7.0, W),
]

# Static LiDAR sensor for validation (placed in hub at origin, height 0.5m)
lidar_model = """
    <!-- 2D LiDAR for sensor validation -->
    <model name="lidar_sensor">
      <static>true</static>
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><cylinder><radius>0.05</radius><length>0.1</length></cylinder></geometry>
          <material><ambient>1 0.2 0.2 1</ambient><diffuse>1 0.2 0.2 1</diffuse></material>
        </visual>
        <sensor name="lidar" type="gpu_lidar">
          <always_on>true</always_on>
          <update_rate>10</update_rate>
          <topic>/scan</topic>
          <ray>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.14159</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.12</min>
              <max>20.0</max>
              <resolution>0.01</resolution>
            </range>
          </ray>
          <plugin name="ignition::gazebo::systems::Sensors" filename="libignition-gazebo-sensors-system.so"/>
        </sensor>
      </link>
    </model>"""

sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="proxy_map">

    <physics name="default" type="ode">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <plugin filename="libignition-gazebo-physics-system.so"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-user-commands-system.so"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="libignition-gazebo-sensors-system.so"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="libignition-gazebo-contact-system.so"
            name="ignition::gazebo::systems::Contact"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation>
        <range>1000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Floor: thin box (plane ignores material in Ignition renderer)
         Frictionless (mu=0) + min_depth=0.001 prevents degenerate LCP constraints -->
    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 -0.005 0 0 0</pose>
      <link name="link">
        <collision name="floor_col">
          <geometry><box><size>200 200 0.01</size></box></geometry>
          <surface>
            <friction><ode><mu>0</mu><mu2>0</mu2></ode></friction>
            <contact><ode><min_depth>0.001</min_depth></ode></contact>
          </surface>
        </collision>
        <visual name="vis">
          <geometry><box><size>200 200 0.01</size></box></geometry>
          <material>
            <ambient>0.05 0.35 0.85 1</ambient>
            <diffuse>0.05 0.35 0.85 1</diffuse>
            <specular>0.08 0.08 0.08 1</specular>
            <pbr>
              <metal>
                <albedo>0.05 0.35 0.85 1</albedo>
                <metalness>0.0</metalness>
                <roughness>0.9</roughness>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>

{"".join(walls)}
{lidar_model}

  </world>
</sdf>
"""

import xml.etree.ElementTree as ET, io, sys

try:
    ET.parse(io.StringIO(sdf))
    print("XML valid ✓")
except ET.ParseError as e:
    print(f"XML ERROR: {e}")
    sys.exit(1)

with open("src/social_nav_env/worlds/proxy_map.sdf", "w") as f:
    f.write(sdf)
print("proxy_map.sdf written with full collision geometry + LiDAR sensor.")
