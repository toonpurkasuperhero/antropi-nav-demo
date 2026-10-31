#!/bin/bash
# Usage: go.sh X Y [QZ QW]   -> goal in the MAP frame (default: facing east)
# Sends the goal, prints Nav2's final status, then measures the real distance left.
X=${1:?usage: go.sh X Y [QZ QW]}; Y=${2:?usage: go.sh X Y [QZ QW]}
QZ=${3:-0.0}; QW=${4:-1.0}
echo "Sending goal ($X, $Y) ..."
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {z: $QZ, w: $QW}}}}" \
  | grep -E "Goal|status"
sleep 4
read nx ny < <(timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -m1 "Translation" | tr -d '[],' | awk '{print $3, $4}')
python3 - "$X" "$Y" "$nx" "$ny" << 'PY'
import sys, math
try:
    gx, gy, nx, ny = map(float, sys.argv[1:5])
except ValueError:
    sys.exit("Could not read the robot position (is Nav2 running?).")
d = math.hypot(gx - nx, gy - ny)
verdict = "ARRIVED (within 0.25 m)" if d <= 0.25 else "NEAR (within 0.5 m)" if d <= 0.5 else "NOT THERE"
print(f"Robot at ({nx:.2f}, {ny:.2f}), goal ({gx}, {gy}): {d:.2f} m away -> {verdict}")
PY
