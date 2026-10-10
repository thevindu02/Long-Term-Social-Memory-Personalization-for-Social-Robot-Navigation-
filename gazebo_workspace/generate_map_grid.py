#!/usr/bin/env python3
import struct
import math
import os

RESOLUTION = 0.05
WIDTH_M = 40.0
HEIGHT_M = 40.0
W = int(WIDTH_M / RESOLUTION)
H = int(HEIGHT_M / RESOLUTION)

# 254 = free space, 0 = occupied, 205 = unknown
grid = [[254 for _ in range(W)] for _ in range(H)]

def draw_wall(cx, cy, sx, sy):
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

W_THICK = 0.3
draw_wall(-3.5, 5.0, 3.0, W_THICK)
draw_wall(3.5, 5.0, 3.0, W_THICK)
draw_wall(-3.5, -5.0, 3.0, W_THICK)
draw_wall(3.5, -5.0, 3.0, W_THICK)
draw_wall(5.0, 3.5, W_THICK, 3.0)
draw_wall(5.0, -3.5, W_THICK, 3.0)
draw_wall(-5.0, 3.5, W_THICK, 3.0)
draw_wall(-5.0, -3.5, W_THICK, 3.0)

draw_wall(-2.0, 10.0, W_THICK, 10.0)
draw_wall(2.0, 10.0, W_THICK, 10.0)
draw_wall(0.0, 15.0, 4.0, W_THICK)

draw_wall(-2.0, -10.0, W_THICK, 10.0)
draw_wall(2.0, -10.0, W_THICK, 10.0)
draw_wall(0.0, -15.0, 4.0, W_THICK)

draw_wall(10.0, -2.0, 10.0, W_THICK)
draw_wall(6.0, 2.0, 2.0, W_THICK)
draw_wall(12.0, 2.0, 6.0, W_THICK)
draw_wall(15.0, 0.0, W_THICK, 4.0)

draw_wall(-10.0, 2.0, 10.0, W_THICK)
draw_wall(-10.0, -2.0, 10.0, W_THICK)
draw_wall(-15.0, 0.0, W_THICK, 4.0)

draw_wall(7.0, 6.0, W_THICK, 8.0)
draw_wall(14.0, 6.0, W_THICK, 8.0)
draw_wall(10.5, 10.0, 7.0, W_THICK)

# Simple floodfill for the unknown region (borders)
def flood_fill(sx, sy, target, replace):
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

flood_fill(0, 0, 254, 205)

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

print("Generated proxy_map.pgm and proxy_map.yaml in", out_dir)
