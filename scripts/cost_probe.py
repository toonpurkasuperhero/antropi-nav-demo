#!/usr/bin/env python3
"""Prints what the LOCAL costmap looks like around the robot (robot = window centre)."""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class Probe(Node):
    def __init__(self):
        super().__init__("cost_probe")
        self.done = False
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.cb, 1)

    def cb(self, m):
        w, h, res = m.info.width, m.info.height, m.info.resolution
        cx, cy = w // 2, h // 2
        d = m.data
        centre = d[cy * w + cx]
        near_max, nearest = 0, None
        n99 = n100 = 0
        for y in range(h):
            for x in range(w):
                v = d[y * w + x]
                if v < 0:
                    continue
                dist = math.hypot(x - cx, y - cy) * res
                if dist <= 0.30:
                    near_max = max(near_max, v)
                if v >= 99:
                    n99 += 1
                    if v == 100:
                        n100 += 1
                    if nearest is None or dist < nearest:
                        nearest = dist
        print(f"Local costmap: {w}x{h} cells, {res} m/cell")
        print(f"Cost under robot centre : {centre}   (0 = free, 99/100 = blocked)")
        print(f"Highest cost within 0.30 m of robot: {near_max}")
        print(f"Blocked cells in window : {n99}  (lethal: {n100})")
        print("Nearest blocked cell    : " +
              (f"{nearest:.2f} m from robot" if nearest is not None else "none"))
        if near_max >= 99:
            print("-> VERDICT: robot is INSIDE a blocked zone. Controller refuses to move.")
        elif nearest is not None and nearest < 0.45:
            print("-> VERDICT: an obstacle is touching the robot's safety margin.")
        else:
            print("-> VERDICT: costmap is clear around the robot (problem is elsewhere).")
        self.done = True


rclpy.init()
n = Probe()
for _ in range(100):
    rclpy.spin_once(n, timeout_sec=0.1)
    if n.done:
        break
if not n.done:
    print("No costmap received in 10 s - is the simulation running and Nav2 active?")
n.destroy_node()
rclpy.shutdown()
