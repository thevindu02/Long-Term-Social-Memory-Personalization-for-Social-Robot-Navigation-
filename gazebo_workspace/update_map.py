import sys

walls = [
    # Hub Outer Corners (creating a 10x10 hub from -5 to 5)
    # North Corridor Walls (y=5 to y=15, x=-2 and x=2)
    ("north_corr_left", -2, 10, 0.2, 10),
    ("north_corr_right", 2, 10, 0.2, 10),
    # South Corridor Walls (y=-5 to y=-15, x=-2 and x=2)
    ("south_corr_left", -2, -10, 0.2, 10),
    ("south_corr_right", 2, -10, 0.2, 10),
    # East Corridor Walls (x=5 to x=15, y=-2 and y=2)
    ("east_corr_top", 10, 2, 10, 0.2),
    ("east_corr_bot", 10, -2, 10, 0.2),
    # West Corridor Walls (x=-5 to x=-15, y=-2 and y=2)
    ("west_corr_top", -10, 2, 10, 0.2),
    ("west_corr_bot", -10, -2, 10, 0.2),
    
    # Hub corner connections
    ("hub_nw", -3.5, 3.5, 3, 3), # solid block or walls? Let's use walls.
    ("hub_nw_h", -3.5, 5, 3, 0.2),
    ("hub_nw_v", -5, 3.5, 0.2, 3),
    
    ("hub_ne_h", 3.5, 5, 3, 0.2),
    ("hub_ne_v", 5, 3.5, 0.2, 3),
    
    ("hub_sw_h", -3.5, -5, 3, 0.2),
    ("hub_sw_v", -5, -3.5, 0.2, 3),
    
    ("hub_se_h", 3.5, -5, 3, 0.2),
    ("hub_se_v", 5, -3.5, 0.2, 3),

    # Add a large room attached to North-East (from x=2 to x=12, y=5 to y=15)
    ("room_ne_top", 7, 15, 10, 0.2),
    ("room_ne_right", 12, 10, 0.2, 10),
    ("room_ne_door_wall1", 4.5, 5, 5, 0.2), # Leaves a gap between x=2 and x=4.5? No, the hub wall is at y=5.
    
    # Closing the corridor ends
    ("north_end", 0, 15, 4, 0.2),
    ("south_end", 0, -15, 4, 0.2),
    ("east_end", 15, 0, 0.2, 4),
    ("west_end", -15, 0, 0.2, 4),
]

sdf_walls = ""
for name, cx, cy, w, h in walls:
    sdf_walls += f"""      <link name="{name}">
        <pose>{cx} {cy} 1 0 0 0</pose>
        <collision name="col">
          <geometry><box><size>{w} {h} 2</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>{w} {h} 2</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient></material>
        </visual>
      </link>
"""

print(sdf_walls)
