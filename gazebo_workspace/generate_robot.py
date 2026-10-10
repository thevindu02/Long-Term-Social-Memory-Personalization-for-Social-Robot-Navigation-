#!/usr/bin/env python3
import os

sdf = """<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="simple_robot">
    <pose>0 0 0.15 0 0 0</pose>
    
    <link name="base_link">
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <iyy>0.1</iyy>
          <izz>0.1</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry><cylinder><radius>0.2</radius><length>0.2</length></cylinder></geometry>
        <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><cylinder><radius>0.2</radius><length>0.2</length></cylinder></geometry>
      </collision>
      
      <!-- Caster wheel to keep it from falling over -->
      <visual name="caster_visual">
        <pose>-0.15 0 -0.1 0 0 0</pose>
        <geometry><sphere><radius>0.05</radius></sphere></geometry>
        <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material>
      </visual>
      <collision name="caster_collision">
        <pose>-0.15 0 -0.1 0 0 0</pose>
        <geometry><sphere><radius>0.05</radius></sphere></geometry>
        <surface>
          <friction><ode><mu>0</mu><mu2>0</mu2></ode></friction>
        </surface>
      </collision>
    </link>

    <link name="left_wheel">
      <pose>0.1 0.25 -0.05 -1.5707 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
    </link>

    <link name="right_wheel">
      <pose>0.1 -0.25 -0.05 -1.5707 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
    </link>

    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_wheel</child>
      <axis><xyz>0 0 1</xyz></axis>
    </joint>

    <joint name="right_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>right_wheel</child>
      <axis><xyz>0 0 1</xyz></axis>
    </joint>

    <link name="lidar_link">
      <pose>0 0 0.15 0 0 0</pose>
      <inertial><mass>0.1</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>
      <visual name="vis">
        <geometry><cylinder><radius>0.05</radius><length>0.05</length></cylinder></geometry>
        <material><ambient>0.2 0.8 0.2 1</ambient><diffuse>0.2 0.8 0.2 1</diffuse></material>
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
      </sensor>
    </link>
    
    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_link</child>
    </joint>

    <plugin
      filename="libignition-gazebo-diff-drive-system.so"
      name="ignition::gazebo::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.5</wheel_separation>
      <wheel_radius>0.1</wheel_radius>
      <odom_publish_frequency>20</odom_publish_frequency>
      <topic>/cmd_vel</topic>
      <odom_topic>/odom</odom_topic>
      <tf_topic>/tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
    </plugin>
  </model>
</sdf>
"""

out_dir = "src/social_nav_env/models/simple_robot"
os.makedirs(out_dir, exist_ok=True)
with open(os.path.join(out_dir, "model.sdf"), 'w') as f:
    f.write(sdf)

print("Generated simple_robot SDF in", out_dir)
