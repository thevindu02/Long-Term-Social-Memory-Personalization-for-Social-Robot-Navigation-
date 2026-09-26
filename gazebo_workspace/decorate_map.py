import os
import re

sdf_path = "src/social_nav_env/worlds/proxy_map.sdf"
with open(sdf_path, 'r') as f:
    content = f.read()

# Change wall color to an off-white/plaster color
content = content.replace('<ambient>0.2 0.2 0.2 1</ambient>', '<ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.1 0.1 0.1 1</specular>')
content = content.replace('<ambient>0.6 0.6 0.6 1</ambient>', '<ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.1 0.1 0.1 1</specular>')

# Change floor color to a light blue tile-like color
content = content.replace('<ambient>0.8 0.8 0.8 1</ambient>\n            <diffuse>0.8 0.8 0.8 1</diffuse>', '<ambient>0.6 0.7 0.8 1</ambient>\n            <diffuse>0.6 0.7 0.8 1</diffuse>')

# Remove old invalid props
start_idx = content.find('<!-- Hospital Furniture from Gazebo Fuel -->')
if start_idx != -1:
    content = content[:start_idx] + "</world>"

props = """
    <!-- Hospital Furniture from Gazebo Fuel -->
    <!-- Reception area in the central hub -->
    <include>
      <name>reception_desk</name>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/BedsideTable</uri>
      <pose>0 2 0 0 0 0</pose>
    </include>
    <include>
      <name>reception_chair</name>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair</uri>
      <pose>0 3 0 0 0 3.14159</pose>
    </include>

    <!-- Hospital Beds in the NE Room -->
    <include>
      <name>bed_1</name>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/CGMClassic</uri>
      <pose>10 8 0 0 0 0</pose>
    </include>
    <include>
      <name>bed_2</name>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/CGMVanguard</uri>
      <pose>10 4 0 0 0 0</pose>
    </include>
    
    <!-- Corridor IV Stands and Chairs -->
    <include>
      <name>iv_stand_1</name>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/IVStand</uri>
      <pose>-1.5 -8 0 0 0 1.5708</pose>
    </include>
    <include>
      <name>chair_2</name>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair</uri>
      <pose>-8 1.5 0 0 0 0</pose>
    </include>
"""

# Insert props before the closing </world> tag
content = content.replace('</world>', props + '\n  </world>')

with open(sdf_path, 'w') as f:
    f.write(content)
print("Map decorated successfully.")
