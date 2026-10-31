#!/bin/bash
# Compares true robot position (Gazebo) with Nav2's belief (map frame).
# Map origin = charger = world (-8.4, -5.5).
read tx ty < <(gz model -m turtlebot3_waffle -p 2>/dev/null | grep -A1 "Pose" | tail -1 | tr -d '[]' | awk '{print $1, $2}')
read nx ny < <(timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -m1 "Translation" | tr -d '[],' | awk '{print $3, $4}')
python3 - "$tx" "$ty" "$nx" "$ny" << 'PY'
import sys, math
try:
    tx, ty, nx, ny = map(float, sys.argv[1:5])
except ValueError:
    sys.exit("Could not read a position. Is the simulation running?")
ex, ey = tx + 8.4, ty + 5.5
err = math.hypot(nx - ex, ny - ey)
print(f"True (map frame): ({ex:.2f}, {ey:.2f})   Nav2 believes: ({nx:.2f}, {ny:.2f})")
print(f"Error: {err:.2f} m ->", "GOOD" if err < 0.3 else "OK" if err < 0.6 else "LOST - reset the pose")
PY
