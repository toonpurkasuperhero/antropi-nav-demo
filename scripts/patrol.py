#!/usr/bin/env python3
"""Antropi CNC cell patrol: the AGV visits stations one after another using Nav2.

Usage:
  patrol.py [--init] demo            short route for the video
  patrol.py [--init] shift           every station
  patrol.py [--init] RAW CNC3 QC     any list of station names

--init  first tells Nav2 the robot is at the charger (only use on a fresh launch).
Needs Nav2 running and the map origin at the charger (see config/stations.yaml).
"""
import math
import os
import sys
import time

import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATIONS_FILE = os.path.join(PKG, "config", "stations.yaml")
DWELL = 2.0   # seconds "loading / unloading" at each station
MISSIONS = {
    "demo": ["RAW", "CNC2", "CNC7", "QC", "FG", "CHARGER"],
    "shift": ["RAW", "CNC1", "CNC2", "CNC3", "CNC4", "CNC5", "CNC6", "CNC7",
              "CNC8", "QC", "FG", "SCRAP", "CHARGER"],
}
STATUS = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}


def load_stations(path=STATIONS_FILE):
    data = yaml.safe_load(open(path))
    if "world_origin_of_map" not in data:
        sys.exit("stations.yaml is an old version (world frame). Re-run scripts/generate_world.py.")
    return data["stations"]


def expand_mission(args, stations):
    names = MISSIONS[args[0]] if len(args) == 1 and args[0] in MISSIONS else args
    unknown = [n for n in names if n not in stations]
    if unknown:
        sys.exit(f"Unknown station(s): {unknown}. Known: {sorted(stations)}")
    return names


def make_pose(x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"      # stamp left at 0 = 'use latest transform'
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


class Patrol(Node):
    def __init__(self):
        super().__init__("antropi_patrol")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.goal_handle = None

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def set_initial_pose(self):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.pose.pose.orientation.w = 1.0
        m.pose.covariance[0] = 0.25
        m.pose.covariance[7] = 0.25
        m.pose.covariance[35] = 0.0685
        self.spin_for(1.0)
        for _ in range(6):
            self.init_pub.publish(m)
            self.spin_for(0.5)
        print("Initial pose sent (charger = map origin). Waiting 5 s for AMCL ...")
        self.spin_for(5.0)

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def goto(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y, yaw)
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        self.goal_handle = fut.result()
        if not self.goal_handle.accepted:
            return "REJECTED"
        res = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return STATUS.get(res.result().status, f"STATUS_{res.result().status}")


def main():
    args = [a for a in sys.argv[1:] if a != "--init"]
    if not args:
        sys.exit(__doc__)
    stations = load_stations()
    names = expand_mission(args, stations)
    rclpy.init()
    node = Patrol()
    if not node.client.wait_for_server(timeout_sec=15.0):
        sys.exit("Nav2 action server not found. Is Nav2 running and active?")
    if "--init" in sys.argv:
        node.set_initial_pose()
    rows = []
    try:
        for name in names:
            s = stations[name]
            print(f"\n-> Driving to {name} ({s['x']}, {s['y']}) ...")
            t0 = time.time()
            status = node.goto(s["x"], s["y"], s["yaw"])
            if status != "SUCCEEDED":
                print(f"   {status} - retrying once")
                status = node.goto(s["x"], s["y"], s["yaw"])
            node.spin_for(1.0)
            xy = node.robot_xy()
            dist = math.hypot(s["x"] - xy[0], s["y"] - xy[1]) if xy else float("nan")
            rows.append((name, status, dist, time.time() - t0))
            print(f"   {status}, {dist:.2f} m from station, {time.time() - t0:.0f} s")
            if status == "SUCCEEDED":
                print(f"   loading/unloading at {name} ...")
                node.spin_for(DWELL)
    except KeyboardInterrupt:
        print("\nStopped by user - cancelling current goal.")
        if node.goal_handle:
            node.goal_handle.cancel_goal_async()
    print("\n=== SUMMARY ===")
    print(f"{'station':<9}{'status':<11}{'dist (m)':<10}{'time (s)':<8}")
    for n, st, d, t in rows:
        print(f"{n:<9}{st:<11}{d:<10.2f}{t:<8.0f}")
    ok = sum(1 for r in rows if r[1] == "SUCCEEDED")
    print(f"{ok}/{len(rows)} stations reached")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok == len(rows) else 1)


if __name__ == "__main__":
    main()
