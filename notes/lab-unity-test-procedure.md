# Testing SUPER on the lab stack (Unity sim)

One container, `ws-palio`, runs everything for a Unity test: the Unity endpoint, the lab dynamics simulator, FAST_LIO, SUPER's `fsm_node`, the `quad_super_waypoint_control` state machine and the keyboard. `~/ws_palio/src/SUPER/` links three packages from this repo: `super_planner`, `rog_map` and `quadrotor_msgs`. The `ws-super` container (`docker/noetic/super.sh`, whole repo) is only for SUPER's own demos and unit tests (`click_perception_demo.launch`, `test_perception_cost`, the benchmarks); nothing in this procedure needs it.

Interface details: `GeneralCode/ROS/hiperlab_rostools/src/QuadSuperWaypointControl/README.md`. All `palio.sh` commands run on the host from the UnitySim repo (`cd /data/Repos/UnitySim`).

## One-time setup (and after changing the Docker image)

```bash
bash docker/noetic/palio.sh init      # creates ~/ws_palio/src/SUPER/{super_planner,rog_map,quadrotor_msgs}
bash docker/noetic/palio.sh image     # adds rosfmt, libdw, yaml-cpp for SUPER
docker rm -f ws-palio && bash docker/noetic/palio.sh up   # recreate the container so /data/Repos/SUPER is mounted
bash docker/noetic/palio.sh build     # builds the lab packages and the three SUPER packages
```

Afterwards only `palio.sh build` is needed after C++ changes (`palio.sh build super_planner` or `hiperlab_rostools` to limit it). YAML and launch edits need no rebuild. Edits under `~/ws_palio/src/SUPER/...` are edits in this repo.

## Choosing the endpoint for an assembled scene

The route is the `waypoints` list in `GeneralCode/ROS/hiperlab_rostools/config/quad_super_waypoint_control_config.yaml`, rows of `[x, y, z]` or `[x, y, z, yaw]`, in metres and radians. With `waypoints_relative_to_start: true` (default) each row is an offset from the drone's position when Start is accepted, in FAST_LIO's `camera_init` frame.

How that frame relates to the Unity scene:

- The lab simulator spawns the drone at its origin with zero yaw, and FAST_LIO's `camera_init` is that start pose. So `camera_init` is the ROS world frame of the simulator: origin where the drone sits when you press Play, x forward along the drone's initial heading, y left, z up.
- Unity follows the simulator pose through the ROS-TCP connector's FLU conversion. For a point at Unity coordinates `(X, Y, Z)`, with the Drone object spawning at the Unity origin: `ROS x = Z`, `ROS y = -X`, `ROS z = Y`. A target 20 m ahead and 5 m to the right in Unity (`X = 5, Z = 20`) is the waypoint `[20, -5, 1.0]`. If the Drone object sits somewhere else when Play starts, subtract its Unity position first.
- The drone flies with body -X toward the goal (`heading_offset_rad: pi`), so the LiDAR, mounted rear-facing, looks forward. In Unity the drone appears to fly backwards; that is intended.
- Height is absolute: `z = 1.0` means 1 m above the spawn height, independent of `flight/takeoff_height`.
- Optional 4th column: the body yaw to settle to while dwelling at that waypoint (`planner/use_waypoint_yaw`). Leave it out or `0.0` otherwise.

Constraints on an endpoint:

- Inside the lab safety box, `safety/min_corner` and `max_corner` (default 40 m each way).
- Inside SUPER's map: `rog_map/map_size` is 50 x 50 x 6 m and slides with the drone, so far goals are fine (SUPER plans receding-horizon, `planning_horizon: 7.0`), but the goal height must sit between `virtual_ground_height` (-0.3 m) and `virtual_ceil_height` (3.0 m) in `camera_init`.
- Not inside an obstacle. SUPER moves a goal to the nearest free inflated cell within 3 m and drops it otherwise; the lab node then resends and finally hovers. Put endpoints in open space, about a metre from walls.
- Several rows make a route; the drone stops at each one. `flight/land_at_end: true` lands after the last.

## Flight procedure

Terminals 1 to 3 (host):

```bash
bash docker/noetic/palio.sh endpoint        # then press Play in Unity; the connector arrows turn blue
bash docker/noetic/palio.sh shell           # rosrun hiperlab_rostools simulator 113
bash docker/noetic/palio.sh shell           # roslaunch fast_lio mapping_unitysim.launch
```

Let FAST_LIO initialise with the drone still, then in any shell:

```bash
rostopic hz /Odometry_imu /cloud_registered /plane_normals
rostopic info /Odometry_imu     # exactly one publisher: laserMapping (keep the simulator's default simVIO)
```

Terminal 4, SUPER. Pick vanilla or perception-aware (next section):

```bash
bash docker/noetic/palio.sh super config:=lab_unity_smooth_ros1.yaml   # vanilla SUPER
bash docker/noetic/palio.sh super                                      # SUPER + perception cost
```

It prints `No odom` until FAST_LIO streams, then `Current state: INIT`. Check `rostopic echo /planning/fsm_state` shows `INIT`.

Terminal 5, RViz (host desktop terminal): `bash docker/noetic/palio.sh rviz-super` opens SUPER's layout with fixed frame `camera_init` (occupancy map, A* path, committed trajectory, corridors, perception cost layer). `palio.sh rviz-planner` is the lab layout (cloud, plane normals, reference). Either works; RViz is not required for the flight.

Terminals 6 and 7, controller and keyboard:

```bash
bash docker/noetic/palio.sh shell           # roslaunch hiperlab_rostools quad_super_waypoint_control.launch vehicle_id:=113 estimator:=lio
bash docker/noetic/palio.sh shell           # rosrun hiperlab_hardware keyboard_control
```

Press `s`, release, then `s` again to take off. `b` requests landing at any time. Keep the keyboard node running: it is also the joystick heartbeat. Service-only alternative: `no_joystick:=true` on the launch and `rosservice call /quad_super_waypoint_control113/start "{}"` (`land`, `kill` likewise).

## What to watch

```bash
rostopic echo /quad_super_waypoint_control113/state
rostopic echo /quad_super_waypoint_control113/planner_state
rostopic hz /planning/ref_traj
```

- Stages: WAIT, SPOOL, TAKEOFF, SEND_GOAL, PLANNER, then DWELL and LANDING (or HOVER). SEND_GOAL should last well under a second. If it sits there with SUPER in WAIT_GOAL, SUPER is dropping the goal: an occupied goal cell, a goal outside the map height band, or a goal within 0.1 m of the drone. The node resends every `planner/goal_resend_sec` and gives up after `planner/max_goal_attempts`.
- At the SEND_GOAL to PLANNER handover the drone must not spin 180 degrees. If it does, `super_planner/yaw_offset_rad` (SUPER yaml) and `waypoint_control/heading_offset_rad` (lab yaml) differ.
- `/planning/ref_traj` runs at 100 Hz while SUPER is in FOLLOW_TRAJ and is silent otherwise. Short gaps are normal when SUPER re-plans from rest; the node holds position through them.
- Tracking: `rqt_plot` with `reference_state113` against `estimator113` (`palio.sh gui rqt_plot`).
- Arrival: the node accepts a stop within `planner/arrival_tolerance` (0.5 m) of the waypoint, because SUPER snaps goals to free cells and stops within `fsm/goal_reach_dis` (0.3 m). A warning prints the actual distance.

Failure drills once a normal flight works: Ctrl+C the SUPER terminal mid-leg (hover after `planner/hold_timeout`, 2 s); stop FAST_LIO (EMERGENCY within 0.5 s); `rosservice call /quad_super_waypoint_control113/land "{}"` from any stage.

## Vanilla SUPER versus SUPER with the perception cost

Both use the same lab node, route and launch; only the SUPER config differs. The two files are identical except `astar/perception/enable`:

| | Command | Config |
|---|---|---|
| Vanilla | `palio.sh super config:=lab_unity_smooth_ros1.yaml` | `super_planner/config/lab_unity_smooth_ros1.yaml` |
| Perception-aware | `palio.sh super` | `super_planner/config/lab_unity_ros1.yaml` |

Procedure:

1. Fly the route vanilla first. It validates the whole chain and gives the baseline path.
2. Fly it once with the perception config unchanged and watch `/fsm_node/visualization/perception_logdet` in `rviz-super` (PointCloud2, intensity = log det). Note the typical value in well-observed areas and the value in the featureless part of your scene. Set `astar/perception/logdet_ref` in `lab_unity_ros1.yaml` just below the typical value and `logdet_scale` to roughly the drop you saw, so only the weak region gets a cost (the study notes: log det barely moves with plane density and drops sharply with rank deficiency, so the scene needs a region that removes a direction of information, such as a long blank wall or open ground).
3. Fly again with the perception config and compare against the vanilla run:
   - `/fsm_node/visualization/frontend_path` (A* path) and `committed_traj` in RViz: the perception run should detour around the weak region.
   - The flown path from `estimator113`, and the LIO quality: FAST_LIO's odometry against `/simulator_truth113` (ground truth), which is the real metric.
   - SUPER's log line `[Perception] grid ... cells penalized, logdet min..max` every 5 s, and A* search time if `super_planner/print_log` is on.
4. Record each run for offline comparison:

```bash
rosbag record -O vanilla.bag /simulator_truth113 /Odometry_imu /estimator113 /reference_state113 /planning/goal /planning/fsm_state /fsm_node/visualization/frontend_path /fsm_node/visualization/committed_traj /fsm_node/visualization/perception_logdet /quad_super_waypoint_control113/state
```

For the same comparison against the lab's own planner, fly the route with `quad_lidar_waypoint_control` and `planner_specs/k_lidar` on and off (its yaml) and compare the same topics.

## Speed

`traj_opt/boundary/max_vel` is 1.5 m/s in both lab configs for the first flights and the lab yaml's `trajectory/max_speed` matches it (that one only warns). Raise them together, along with `max_acc`.

## Two-container variant

`bash docker/noetic/super.sh lab ...` in this repo still launches the same `lab_fsm.launch` from `ws-super` against the host master, if `ws-palio` was built without SUPER. Not needed with the setup above.

## Automated four-case comparison

`GeneralCode/lio_comparison/compare_planners.sh` flies the route of the LiDAR controller YAML four times in the current scene (LiDAR planner with `lidar_aware` off and on, SUPER with the perception cost off and on), records a bag per flight, and produces the top-down trajectories, the X/Y/Z/yaw RMSE figure and a table. Unity in Play, container running, nothing else started by hand:

```bash
bash /data/Repos/GeneralCode/lio_comparison/compare_planners.sh --k-lidar 1.0
```

Details and options in `GeneralCode/lio_comparison/README.md` ("Planner comparison").
