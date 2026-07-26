import os
import json

map_name = "hospital_ward"
base_dir = f"submodules/amrl_maps/{map_name}"
template_dir = "submodules/amrl_maps/multienv"

# Define the line segments (walls)
lines = []

# Corridor walls (y=2 and y=-2), with gaps for doors
# Top wall
lines.append(([-15, 2], [-9.5, 2]))
lines.append(([-8.5, 2], [-0.5, 2]))
lines.append(([0.5, 2], [8.5, 2]))
lines.append(([9.5, 2], [15, 2]))

# Bottom wall
lines.append(([-15, -2], [-9.5, -2]))
lines.append(([-8.5, -2], [-0.5, -2]))
lines.append(([0.5, -2], [8.5, -2]))
lines.append(([9.5, -2], [15, -2]))

# Corridor ends (blocking the ends of the hallway)
lines.append(([-15, -2], [-15, 2]))
lines.append(([15, -2], [15, 2]))

# Top Rooms (y=2 to y=7)
lines.append(([-12, 2], [-12, 7]))
lines.append(([-12, 7], [-6, 7]))
lines.append(([-6, 7], [-6, 2]))

lines.append(([-3, 2], [-3, 7]))
lines.append(([-3, 7], [3, 7]))
lines.append(([3, 7], [3, 2]))

lines.append(([6, 2], [6, 7]))
lines.append(([6, 7], [12, 7]))
lines.append(([12, 7], [12, 2]))

# Bottom Rooms (y=-2 to y=-7)
lines.append(([-12, -2], [-12, -7]))
lines.append(([-12, -7], [-6, -7]))
lines.append(([-6, -7], [-6, -2]))

lines.append(([-3, -2], [-3, -7]))
lines.append(([-3, -7], [3, -7]))
lines.append(([3, -7], [3, -2]))

lines.append(([6, -2], [6, -7]))
lines.append(([6, -7], [12, -7]))
lines.append(([12, -7], [12, -2]))

# 1. Create directory structure
os.makedirs(base_dir, exist_ok=True)

# 2. Write vectormap.json
vectormap_json = []
for p0, p1 in lines:
    vectormap_json.append({
        "p0": {"x": str(float(p0[0])), "y": str(float(p0[1]))},
        "p1": {"x": str(float(p1[0])), "y": str(float(p1[1]))}
    })

with open(f"{base_dir}/{map_name}.vectormap.json", "w") as f:
    json.dump(vectormap_json, f)

# 3. Write vectormap.txt
with open(f"{base_dir}/{map_name}.vectormap.txt", "w") as f:
    for p0, p1 in lines:
        f.write(f"{float(p0[0]):.6f}, {float(p0[1]):.6f}, {float(p1[0]):.6f}, {float(p1[1]):.6f}\n")

# 4. Generate scene.xml
scene_xml = '''<?xml version="1.0" encoding="UTF-8"?>
<scenario>
    <!--Obstacles-->
'''
for p0, p1 in lines:
    scene_xml += f'\t<obstacle x1="{float(p0[0]):.6f}" y1="{float(p0[1]):.6f}" x2="{float(p1[0]):.6f}" y2="{float(p1[1]):.6f}"/>\n'

scene_xml += '''
    <!--Way Points-->
{% for i in range(position_count) %}
    <waypoint id="{{ i }}" x="{{ positions[i][0] }}" y = "{{ positions[i][1] }}" r="1" b="0.1"/>
{% endfor %}

{% for i in range(nav_count) %}
    <waypoint id="n{{ i }}" x="{{ nav_map[i][0] }}" y = "{{ nav_map[i][1] }}" r="1" b="0.1"/>
{% endfor %}

    <!-- This Robot Goal Doesn't Matter, but is Required -->
  <waypoint id="robot_goal" x="{{ robot_end[0] }}" y="{{ robot_end[1] }}" r="2"/>
  <waypoint id="robot_start" x="{{ robot_start[0] }}" y="{{ robot_start[1] }}" r="2"/>

  <agent x="{{ robot_start[0] }}" y="{{ robot_start[1] }}" n="1" dx="0" dy="0" type="2">
    <addwaypoint id="robot_start"/>
    <addwaypoint id="robot_goal"/>
  </agent>

  {% for human_position in human_positions %}
  <agent x="{{ human_position[0] }}" y="{{ human_position[1] }}" n="1" dx="{{ dev }}" dy="{{ dev }}" type="0">
    {% for i in range(3, human_position|length) %}
    <addwaypoint id="n{{human_position[i]}}" />
    {% endfor %}
  </agent>
  {% endfor %}

</scenario>
'''
with open(f"{base_dir}/scene.xml", "w") as f:
    f.write(scene_xml)

# 5. Copy and process template files from multienv
files_to_copy = [
    "all_launch.launch", "config_launch.launch", "greedy_launch.launch", 
    "humans.lua", "launch.launch", "pedsim_launch.launch", 
    "pips_launch.launch", "ref_launch.launch", "sim_config.lua",
    "multienv.navigation.json", "multienv.navigation.txt"
]

for file in files_to_copy:
    src_path = os.path.join(template_dir, file)
    dst_name = file.replace("multienv", map_name)
    dst_path = os.path.join(base_dir, dst_name)
    
    if file == "multienv.navigation.json":
        content = '''{
    "nodes": [
        {"id": 0, "loc": {"x": -14.0, "y": 0.0}},
        {"id": 1, "loc": {"x": -9.0, "y": 0.0}},
        {"id": 2, "loc": {"x": 0.0, "y": 0.0}},
        {"id": 3, "loc": {"x": 9.0, "y": 0.0}},
        {"id": 4, "loc": {"x": 14.0, "y": 0.0}},
        {"id": 5, "loc": {"x": -9.0, "y": 4.5}},
        {"id": 6, "loc": {"x": 0.0, "y": 4.5}},
        {"id": 7, "loc": {"x": 9.0, "y": 4.5}},
        {"id": 8, "loc": {"x": -9.0, "y": -4.5}},
        {"id": 9, "loc": {"x": 0.0, "y": -4.5}},
        {"id": 10, "loc": {"x": 9.0, "y": -4.5}}
    ],
    "edges": [
        { "has_automated_door": false, "has_door": false, "has_elevator": false, "has_stairs": false, "max_clearance": 1.0, "max_speed": 2.0, "s0_id": 0, "s1_id": 1 },
        { "has_automated_door": false, "has_door": false, "has_elevator": false, "has_stairs": false, "max_clearance": 1.0, "max_speed": 2.0, "s0_id": 1, "s1_id": 2 },
        { "has_automated_door": false, "has_door": false, "has_elevator": false, "has_stairs": false, "max_clearance": 1.0, "max_speed": 2.0, "s0_id": 2, "s1_id": 3 },
        { "has_automated_door": false, "has_door": false, "has_elevator": false, "has_stairs": false, "max_clearance": 1.0, "max_speed": 2.0, "s0_id": 3, "s1_id": 4 },
        { "has_automated_door": false, "has_door": false, "has_elevator": false, "has_stairs": false, "max_clearance": 1.0, "max_speed": 2.0, "s0_id": 1, "s1_id": 5 },
        { "has_automated_door": false, "has_door": false, "has_elevator": false, "has_stairs": false, "max_clearance": 1.0, "max_speed": 2.0, "s0_id": 2, "s1_id": 6 },
        { "has_automated_door": false, "has_door": false, "has_elevator": false, "has_stairs": false, "max_clearance": 1.0, "max_speed": 2.0, "s0_id": 3, "s1_id": 7 },
        { "has_automated_door": false, "has_door": false, "has_elevator": false, "has_stairs": false, "max_clearance": 1.0, "max_speed": 2.0, "s0_id": 1, "s1_id": 8 },
        { "has_automated_door": false, "has_door": false, "has_elevator": false, "has_stairs": false, "max_clearance": 1.0, "max_speed": 2.0, "s0_id": 2, "s1_id": 9 },
        { "has_automated_door": false, "has_door": false, "has_elevator": false, "has_stairs": false, "max_clearance": 1.0, "max_speed": 2.0, "s0_id": 3, "s1_id": 10 }
    ]
}'''
    else:
        if not os.path.exists(src_path):
            continue
            
        with open(src_path, "r") as f:
            content = f.read()
        
        # Replace multienv references with hospital_ward
        content = content.replace("multienv", map_name)
    
    with open(dst_path, "w") as f:
        f.write(content)

print(f"Successfully generated map files in {base_dir}")
