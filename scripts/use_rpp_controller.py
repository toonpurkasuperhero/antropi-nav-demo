import shutil, sys, os
p = os.path.expanduser("~/ros2_ws/src/antropi_nav_demo/config/nav2_params.yaml")
shutil.copy(p, p + ".mppi_backup")
L = open(p).read().split("\n")
i = next((k for k, l in enumerate(L) if l == "    FollowPath:"), None)
if i is None:
    sys.exit("ERROR: could not find '    FollowPath:' - file left unchanged")
j = next((k for k in range(i + 1, len(L)) if L[k] and L[k][0] not in " \t#"), len(L))
new = '''    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.35
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.0
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.2
      use_fixed_curvature_lookahead: false
      curvature_lookahead_dist: 0.25
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      allow_reversing: false
      max_angular_accel: 3.2
      max_robot_pose_search_dist: 10.0
'''.split("\n")
L[i:j] = new
open(p, "w").write("\n".join(L))
print(f"OK: replaced lines {i+1}-{j} (MPPI) with RPP. Backup: {p}.mppi_backup")
