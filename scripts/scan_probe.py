#!/usr/bin/env python3
"""Checks whether /scan carries real laser readings."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Probe(Node):
    def __init__(self):
        super().__init__("scan_probe")
        self.n = 0
        self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)

    def cb(self, m):
        self.n += 1
        if self.n < 3:
            return
        good = [r for r in m.ranges if math.isfinite(r) and m.range_min <= r <= m.range_max]
        print(f"frame_id: {m.header.frame_id}   beams: {len(m.ranges)}   valid readings: {len(good)}")
        if good:
            print(f"closest: {min(good):.2f} m   farthest: {max(good):.2f} m")
            print("-> VERDICT: lidar works.")
        else:
            print("-> VERDICT: lidar sends only empty/infinite readings (it sees nothing).")
        raise SystemExit


rclpy.init()
n = Probe()
try:
    for _ in range(100):
        rclpy.spin_once(n, timeout_sec=0.1)
    print("No /scan messages in 10 s -> the lidar is NOT publishing.")
except SystemExit:
    pass
n.destroy_node()
rclpy.shutdown()
