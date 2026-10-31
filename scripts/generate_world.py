#!/usr/bin/env python3
"""Generate the Antropi CNC Cell world for Gazebo Harmonic.

Outputs (relative to the package folder):
  worlds/antropi_cnc_cell.sdf   the Gazebo world
  config/stations.yaml          goal poses (x, y, yaw) for the patrol script
  docs/layout_preview.png       top-down picture (only if matplotlib exists)

Everything is built from simple boxes and cylinders, so no model
downloads are needed. Units: metres. Origin (0,0) = centre of the hall.
"""
import math
import os

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
W, H = 20.0, 14.0          # hall size (x, y)
PI = math.pi

COLORS = {
    "floor": (0.80, 0.80, 0.82), "wall": (0.92, 0.92, 0.94),
    "cnc": (0.10, 0.35, 0.70), "table": (0.55, 0.55, 0.58),
    "rack": (0.95, 0.55, 0.10), "crate": (0.65, 0.45, 0.22),
    "pillar": (0.45, 0.45, 0.48), "hazard": (0.95, 0.80, 0.10),
    "bin": (0.25, 0.55, 0.30), "qc": (0.60, 0.30, 0.65),
    "fg": (0.20, 0.65, 0.55), "charger": (0.85, 0.15, 0.15),
    "lane": (0.10, 0.70, 0.30), "human": (0.98, 0.90, 0.20),
    "future": (0.65, 0.65, 0.70),
}

models = []     # SDF text for each model
solids = []     # (name, kind, x, y, sx, sy) for collision self-check
decals = []     # (kind, x, y, sx, sy) flat floor markings, for the preview


def _mat(kind):
    r, g, b = COLORS[kind]
    return (f"<material><ambient>{r} {g} {b} 1</ambient>"
            f"<diffuse>{r} {g} {b} 1</diffuse></material>")


def box(name, kind, x, y, sx, sy, sz, solid=True, z0=0.0, yaw=0.0):
    """Static box standing on the floor. solid=False -> flat floor marking."""
    col = ""
    if solid:
        col = (f"<collision name='c'><geometry><box><size>{sx} {sy} {sz}"
               f"</size></box></geometry></collision>")
        solids.append((name, kind, x, y, sx, sy))
    else:
        decals.append((kind, x, y, sx, sy))
    models.append(
        f"<model name='{name}'><static>true</static>"
        f"<pose>{x} {y} {z0 + sz / 2} 0 0 {yaw}</pose><link name='link'>{col}"
        f"<visual name='v'><geometry><box><size>{sx} {sy} {sz}</size></box>"
        f"</geometry>{_mat(kind)}</visual></link></model>")


def cyl(name, kind, x, y, r, h):
    solids.append((name, kind, x, y, 2 * r, 2 * r))
    models.append(
        f"<model name='{name}'><static>true</static><pose>{x} {y} {h / 2} 0 0 0</pose>"
        f"<link name='link'><collision name='c'><geometry><cylinder><radius>{r}"
        f"</radius><length>{h}</length></cylinder></geometry></collision>"
        f"<visual name='v'><geometry><cylinder><radius>{r}</radius><length>{h}"
        f"</length></cylinder></geometry>{_mat(kind)}</visual></link></model>")


def line(name, x1, y1, x2, y2, kind="lane", width=0.12):
    """Flat floor line (visual only, the lidar ignores it)."""
    box(name, kind, (x1 + x2) / 2, (y1 + y2) / 2,
        abs(x2 - x1) + width, abs(y2 - y1) + width, 0.006, solid=False)


# ---------------------------------------------------------------- floor, walls
box("floor", "floor", 0, -0.0, W + 0.4, H + 0.4, 0.1, z0=-0.1)
solids.pop()  # the floor is not an obstacle for the self-check
WALL_H = 1.5
box("wall_north", "wall", 0, H / 2 + 0.1, W + 0.4, 0.2, WALL_H)
box("wall_south", "wall", 0, -H / 2 - 0.1, W + 0.4, 0.2, WALL_H)
box("wall_west", "wall", -W / 2 - 0.1, 0, 0.2, H + 0.4, WALL_H)
box("wall_east", "wall", W / 2 + 0.1, 0, 0.2, H + 0.4, WALL_H)

stations = {}

# ---------------------------------------------------------- Bay A (north row)
for i, x in enumerate([-4.5, -2.5, -0.5, 1.5, 3.5], start=1):
    box(f"cnc{i}", "cnc", x, 6.1, 1.4, 1.2, 1.4)
    box(f"cnc{i}_table", "table", x, 4.55, 0.9, 0.4, 0.5)
    box(f"cnc{i}_zone", "human", x, 4.95, 1.4, 1.2, 0.01, solid=False)
    stations[f"CNC{i}"] = (x, 3.9, PI / 2)
for j, x in enumerate([5.5, 7.5], start=1):          # room to grow
    box(f"future{j}", "future", x, 6.1, 1.4, 1.2, 0.01, solid=False)

# ----------------------------------------------- island cells (middle, face N)
for i, x in zip([6, 7, 8], [-2.5, -0.5, 1.5]):
    box(f"cnc{i}", "cnc", x, 0.7, 1.4, 1.2, 1.4)
    box(f"cnc{i}_table", "table", x, 1.55, 0.9, 0.4, 0.5)
    box(f"cnc{i}_bin", "bin", x, -0.45, 0.6, 0.6, 0.6)   # chip bin behind
    stations[f"CNC{i}"] = (x, 2.35, -PI / 2)

# ------------------------------------------------------------- raw store (W)
box("rack1", "rack", -9.2, 5.0, 0.8, 3.0, 1.8)
box("rack2", "rack", -9.2, 1.0, 0.8, 3.0, 1.8)
box("raw_pallet", "crate", -8.4, 3.0, 0.8, 0.8, 0.8)
box("raw_crate1", "crate", -9.2, -1.6, 0.8, 0.8, 0.9)
box("raw_crate2", "crate", -9.2, -2.7, 0.8, 0.8, 0.9)
stations["RAW"] = (-7.4, 3.0, PI)

# ---------------------------------------------------------- QC + finished goods
box("qc_table", "qc", 9.1, 3.1, 1.2, 2.0, 0.9)
stations["QC"] = (7.9, 3.1, 0.0)
box("fg_dock", "fg", 9.1, -2.2, 1.2, 2.0, 0.5)
stations["FG"] = (7.9, -2.2, 0.0)

# ------------------------------------------------------------------- charger
box("charger_pad", "hazard", -8.4, -5.5, 1.4, 1.2, 0.01, solid=False)
box("charger_post", "charger", -9.6, -5.5, 0.3, 0.6, 0.6)
stations["CHARGER"] = (-8.4, -5.5, 0.0)

# ----------------------------------------------------- south area + obstacles
for k, x in enumerate([-3.0, 0.0, 3.0], start=1):
    box(f"wip_rack{k}", "rack", x, -6.5, 2.0, 0.6, 1.6)
box("scrap_container", "bin", 6.5, -5.6, 1.4, 1.0, 0.8)
stations["SCRAP"] = (6.5, -4.3, -PI / 2)
box("forklift_parked", "hazard", -1.0, -4.8, 2.0, 1.0, 1.4)
box("pallet_jack", "hazard", 3.2, -2.2, 1.2, 0.5, 0.6)  # partly blocks lane
for k, (x, y) in enumerate([(-6.0, 0.3), (6.0, 0.3), (-6.0, -4.8),
                            (4.6, -4.8), (-6.0, 6.1), (9.0, 6.1)], start=1):
    cyl(f"pillar{k}", "pillar", x, y, 0.25, 1.5)

# ------------------------------------------------------- floor guide line (loop)
line("loop_prod", -7.0, 3.1, 7.0, 3.1)      # production aisle  (flows east)
line("loop_east", 7.0, 3.1, 7.0, -2.2)      # east connector    (flows south)
line("loop_ret", 7.0, -2.2, -7.0, -2.2)     # return lane       (flows west)
line("loop_west", -7.0, -2.2, -7.0, 3.1)    # west connector    (flows north)

# ------------------------------------------------------------ self-check
problems = []
for i, a in enumerate(solids):
    if a[0].startswith("wall"):
        continue
    for b in solids[i + 1:]:
        if b[0].startswith("wall"):
            continue
        if (abs(a[2] - b[2]) < (a[4] + b[4]) / 2 - 1e-6 and
                abs(a[3] - b[3]) < (a[5] + b[5]) / 2 - 1e-6):
            problems.append(f"OVERLAP: {a[0]} and {b[0]}")
for sname, (gx, gy, _) in stations.items():
    for n, _k, x, y, sx, sy in solids:
        dx = max(abs(gx - x) - sx / 2, 0)
        dy = max(abs(gy - y) - sy / 2, 0)
        if math.hypot(dx, dy) < 0.30:
            problems.append(f"STATION {sname} too close to {n}")

# ------------------------------------------------------------ write the world
sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.7 0.7 0.7 1</ambient>
      <background>0.75 0.8 0.85 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -1.0</direction>
    </light>
    {chr(10).join('    ' + m for m in models)}
  </world>
</sdf>
"""
os.makedirs(os.path.join(PKG, "worlds"), exist_ok=True)
os.makedirs(os.path.join(PKG, "config"), exist_ok=True)
os.makedirs(os.path.join(PKG, "docs"), exist_ok=True)
with open(os.path.join(PKG, "worlds", "antropi_cnc_cell.sdf"), "w") as f:
    f.write(sdf)

# SLAM puts the map origin where the robot starts (the CHARGER, yaw 0), so the
# goals saved for Nav2 are the world coordinates minus the charger position.
cx, cy, _ = stations["CHARGER"]
with open(os.path.join(PKG, "config", "stations.yaml"), "w") as f:
    f.write("# Goal poses in the Nav2 'map' frame: x (m), y (m), yaw (rad)\n"
            "# Valid when the map was recorded starting from the CHARGER pose.\n"
            "# Generated by scripts/generate_world.py - do not edit by hand\n"
            f"world_origin_of_map: {{x: {cx}, y: {cy}, yaw: 0.0}}\n"
            "stations:\n")
    for sname, (x, y, yaw) in stations.items():
        f.write(f"  {sname}: {{x: {round(x - cx, 3)}, y: {round(y - cy, 3)}, "
                f"yaw: {round(yaw, 4)}}}\n")

print(f"World written: {len(models)} models, {len(stations)} stations")
for p in problems:
    print("WARNING:", p)
if not problems:
    print("Self-check passed: no overlaps, all stations have clearance.")

# ------------------------------------------------------------ optional picture
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, ax = plt.subplots(figsize=(12, 8.4))
    for kind, x, y, sx, sy in decals:
        ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy,
                               color=COLORS[kind], alpha=0.6, lw=0))
    for n, kind, x, y, sx, sy in solids:
        ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy,
                               color=COLORS[kind], ec="black", lw=0.6))
    for sname, (x, y, yaw) in stations.items():
        ax.annotate("", xy=(x + 0.5 * math.cos(yaw), y + 0.5 * math.sin(yaw)),
                    xytext=(x, y), arrowprops=dict(arrowstyle="->", color="red", lw=2))
        ax.text(x, y - 0.35, sname, ha="center", va="top", fontsize=8,
                color="red", weight="bold")
    ax.set_xlim(-W / 2 - 0.5, W / 2 + 0.5)
    ax.set_ylim(-H / 2 - 0.5, H / 2 + 0.5)
    ax.set_aspect("equal")
    ax.set_title("Antropi CNC Cell - layout (red = AGV station goals)")
    fig.savefig(os.path.join(PKG, "docs", "layout_preview.png"), dpi=110,
                bbox_inches="tight")
    print("Preview written: docs/layout_preview.png")
except ImportError:
    print("matplotlib not installed - skipped the preview picture (optional)")
