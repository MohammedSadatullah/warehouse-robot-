#!/usr/bin/env python3
"""
Generates warehouse_map.pgm and warehouse_map.yaml
directly from SDF coordinates. No robot movement needed.

Map specs:
  Resolution: 0.05 m/pixel
  World: 20m x 24m  (x: -10..+10, y: -12..+12)
  Image: 400 x 480 pixels

Pixel convention (ROS standard):
  255 = free space (white)
    0 = occupied (black)
  205 = unknown (grey)  -- we set all known space to free
"""

import numpy as np
from PIL import Image
import yaml
import os

# ── Map parameters ────────────────────────────────────────────────────────────
RESOLUTION  = 0.05       # metres per pixel
ORIGIN_X    = -10.0      # world x at pixel col 0
ORIGIN_Y    = -12.0      # world y at pixel row 0

WORLD_W     = 20.0       # metres  x: -10 to +10
WORLD_H     = 24.0       # metres  y: -12 to +12

IMG_W = int(WORLD_W / RESOLUTION)   # 400 pixels
IMG_H = int(WORLD_H / RESOLUTION)   # 480 pixels

FREE     = 255
OCCUPIED = 0
UNKNOWN  = 205

def world_to_pixel(wx, wy):
    """Convert world (x,y) to image (col, row). Row 0 = south in ROS."""
    col = int((wx - ORIGIN_X) / RESOLUTION)
    row = int((wy - ORIGIN_Y) / RESOLUTION)
    return col, row

def fill_rect(img, wx, wy, ww, wh, value, margin=0.0):
    """Fill a rectangle centred at (wx,wy) with size (ww,wh) in world coords."""
    x0 = wx - ww/2 - margin
    x1 = wx + ww/2 + margin
    y0 = wy - wh/2 - margin
    y1 = wy + wh/2 + margin
    c0, r0 = world_to_pixel(x0, y0)
    c1, r1 = world_to_pixel(x1, y1)
    c0 = max(0, min(IMG_W-1, c0))
    c1 = max(0, min(IMG_W-1, c1))
    r0 = max(0, min(IMG_H-1, r0))
    r1 = max(0, min(IMG_H-1, r1))
    img[r0:r1+1, c0:c1+1] = value

# ── Create blank free map ─────────────────────────────────────────────────────
img = np.full((IMG_H, IMG_W), FREE, dtype=np.uint8)

# ── OUTER WALLS ───────────────────────────────────────────────────────────────
WALL_T = 0.3   # wall thickness with margin

# South wall (two sections with 2m gap at centre)
fill_rect(img, -5.1, -12.0, 9.8, WALL_T, OCCUPIED)   # wall_south_L
fill_rect(img,  5.1, -12.0, 9.8, WALL_T, OCCUPIED)   # wall_south_R
# North wall
fill_rect(img,  0.0,  12.0, 20.0, WALL_T, OCCUPIED)
# West wall
fill_rect(img, -10.0,  0.0, WALL_T, 24.0, OCCUPIED)
# East wall
fill_rect(img,  10.0,  0.0, WALL_T, 24.0, OCCUPIED)

# ── SHELF ROWS ────────────────────────────────────────────────────────────────
# Each shelf: 0.5m deep x 2.0m long x positions from SDF
shelves = [
    # Row A  x=-7.5
    (-7.5, 3), (-7.5, 6), (-7.5, 9),
    # Row B  x=-6.0
    (-6.0, 3), (-6.0, 6), (-6.0, 9),
    # Row C  x=+6.0
    ( 6.0, 3), ( 6.0, 6), ( 6.0, 9),
    # Row D  x=+7.5
    ( 7.5, 3), ( 7.5, 6), ( 7.5, 9),
]
for sx, sy in shelves:
    fill_rect(img, sx, sy, 0.5, 2.0, OCCUPIED, margin=0.05)

# ── WORKBENCH (SE corner) ─────────────────────────────────────────────────────
fill_rect(img, 7.5, -8.0, 2.0, 0.8, OCCUPIED, margin=0.05)

# ── CHARGING DOCK ─────────────────────────────────────────────────────────────
fill_rect(img, 0.0, -10.5, 0.6, 0.4, OCCUPIED, margin=0.02)

# ── PEDESTRIAN CYLINDERS (static obstacles, radius 0.22m) ────────────────────
def fill_circle(img, wx, wy, radius, value):
    cx, cy = world_to_pixel(wx, wy)
    r_px   = int(radius / RESOLUTION) + 1
    for dr in range(-r_px, r_px+1):
        for dc in range(-r_px, r_px+1):
            if dr*dr + dc*dc <= r_px*r_px:
                rr = cy + dr
                cc = cx + dc
                if 0 <= rr < IMG_H and 0 <= cc < IMG_W:
                    img[rr, cc] = value

fill_circle(img,  0.0,  -8.0, 0.3, OCCUPIED)   # pedestrian_1
fill_circle(img, -6.75,  2.0, 0.3, OCCUPIED)   # pedestrian_2
fill_circle(img, -8.5,   0.0, 0.3, OCCUPIED)   # pedestrian_3

# ── ROS map convention: flip Y axis ──────────────────────────────────────────
# ROS .pgm maps have row 0 = max Y (north), row H = min Y (south)
img = np.flipud(img)

# ── Save .pgm ─────────────────────────────────────────────────────────────────
out_dir  = os.path.expanduser('~')
pgm_path = os.path.join(out_dir, 'warehouse_map.pgm')
yaml_path= os.path.join(out_dir, 'warehouse_map.yaml')

Image.fromarray(img, mode='L').save(pgm_path)
print(f'Saved: {pgm_path}  ({IMG_W}x{IMG_H} px)')

# ── Save .yaml ────────────────────────────────────────────────────────────────
map_yaml = {
    'image':      'warehouse_map.pgm',
    'resolution':  RESOLUTION,
    'origin':     [ORIGIN_X, ORIGIN_Y, 0.0],
    'negate':      0,
    'occupied_thresh': 0.65,
    'free_thresh':     0.196,
    'mode':       'trinary',
}
with open(yaml_path, 'w') as f:
    yaml.dump(map_yaml, f, default_flow_style=False)
print(f'Saved: {yaml_path}')
print('Done. Load with:')
print('  ros2 run nav2_map_server map_server --ros-args -p map_file_path:=~/warehouse_map.yaml')
