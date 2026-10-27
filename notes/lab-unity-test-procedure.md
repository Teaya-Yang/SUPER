# Testing SUPER on the lab stack (Unity sim)

Procedure for flying SUPER with the lab's `quad_super_waypoint_control` state machine in the Unity simulator. It is the normal Unity runbook (`~/ws_palio/unity-drone-simulator.md`) with SUPER added as one extra process and the new controller in place of the simple one. Interface details: `GeneralCode/ROS/hiperlab_rostools/src/QuadSuperWaypointControl/README.md`.

Two containers share the host ROS master:

| Container | Repo / launcher | Runs |
|---|---|---|
| `ws-palio` | `Unity-Simulator/docker/noetic/palio.sh` | Unity endpoint, `simulator 113`, FAST_LIO, controller, keyboard |
| `ws-super` | `SUPER/docker/noetic/super.sh` | `fsm_node` only (`launch/lab_fsm.launch`) |

## Before flying

- Rebuild after C++ changes: `bash docker/noetic/palio.sh build hiperlab_rostools` (Unity-Simulator repo) and `bash docker/noetic/super.sh build` (SUPER repo). YAML and launch changes need no rebuild.
- Start the lab stack first so the master exists when SUPER comes up.
- Check the waypoint route in `hiperlab_rostools/config/quad_super_waypoint_control_config.yaml` fits the scene. Waypoints are offsets from the takeoff position; the default is the lidar controller's single waypoint 30 m along -X.
- Keep the simulator's default `simVIO` so only FAST_LIO publishes `/Odometry_imu`.

## Terminals 1 to 3: lab stack (host, in the Unity-Simulator repo)

```bash
bash docker/noetic/palio.sh endpoint        # then press Play in Unity
bash docker/noetic/palio.sh shell           # rosrun hiperlab_rostools simulator 113
bash docker/noetic/palio.sh shell           # roslaunch fast_lio mapping_unitysim.launch
```

Let FAST_LIO initialise with the drone still, then check the inputs SUPER consumes:

```bash
rostopic hz /Odometry_imu /cloud_registered /plane_normals
rostopic info /Odometry_imu     # exactly one publisher: laserMapping
```

All three are in `camera_init`, which SUPER treats as its world frame.

## Terminal 4: SUPER (host, in the SUPER repo)

```bash
bash docker/noetic/super.sh lab config:=lab_unity_smooth_ros1.yaml rviz:=true   # cost off for the first flights
```

`rviz:=true` needs a desktop terminal with a display, like the demos; without it the command also works headless. Confirm it is alive:

```bash
bash docker/noetic/super.sh exec rostopic echo /planning/fsm_state   # INIT
```

rog_map's "no point cloud" warnings stop once FAST_LIO streams. In SUPER's RViz set the fixed frame to `camera_init`; the inflated occupancy map should grow around the drone. The launch adds an identity `camera_init -> world` TF so SUPER's markers line up.

## Terminals 5 and 6: controller and keyboard (lab shells)

```bash
roslaunch hiperlab_rostools quad_super_waypoint_control.launch vehicle_id:=113 estimator:=lio
rosrun hiperlab_hardware keyboard_control
```

In the keyboard terminal press `s`, release, then `s` again to take off. `b` requests landing at any time. Keep the keyboard node running: it is also the joystick heartbeat.

Service-only alternative: add `no_joystick:=true` to the launch and call `rosservice call /quad_super_waypoint_control113/start "{}"` (`land`, `kill` likewise).

## What to watch during the flight

```bash
rostopic echo /quad_super_waypoint_control113/state
rostopic echo /quad_super_waypoint_control113/planner_state
rostopic hz /planning/ref_traj
```

- Stages: WAIT, SPOOL, TAKEOFF, SEND_GOAL, PLANNER, then DWELL and LANDING (or HOVER with `flight/land_at_end: false`). SEND_GOAL should last well under a second. If it sits there with SUPER in WAIT_GOAL, SUPER is dropping the goal: usually an occupied goal cell, a goal outside the map, or a goal within 0.1 m of the drone. The node resends every `planner/goal_resend_sec` and gives up after `planner/max_goal_attempts`.
- At the SEND_GOAL to PLANNER handover the drone must not spin 180 degrees. If it does, `super_planner/yaw_offset_rad` (SUPER yaml) and `waypoint_control/heading_offset_rad` (lab yaml) are not equal.
- `/planning/ref_traj` runs at 100 Hz while SUPER is in FOLLOW_TRAJ and is silent otherwise. Short gaps are expected when SUPER re-plans from rest; the node holds position through them.
- Tracking: rqt_plot `reference_state113` against `estimator113`.
- Arrival: the node accepts a stop within `planner/arrival_tolerance` (0.5 m) of the waypoint because SUPER snaps goals to the nearest free inflated cell and stops within its `fsm/goal_reach_dis` (0.3 m). A warning names the actual distance.

## Failure drills, once a normal flight works

- Ctrl+C the SUPER launch mid-leg: the drone stops and hovers after `planner/hold_timeout` (2 s). `planner/on_planner_loss: land` lands instead.
- Stop FAST_LIO: EMERGENCY within 0.5 s (SafetyNet, unchanged).
- `rosservice call /quad_super_waypoint_control113/land "{}"` works from any stage.

## Then the perception cost

```bash
bash docker/noetic/super.sh lab rviz:=true        # lab_unity_ros1.yaml, cost on
```

During a normal flight look at `/fsm_node/visualization/perception_logdet` and set `astar/perception/logdet_ref` in `super_planner/config/lab_unity_ros1.yaml` just below the typical value, so only weak regions get a cost (see the tuning notes in `super-study-notes.md`). Then compare the A* path (`/fsm_node/visualization/frontend_path`) and the flown path against the cost-off run, and against the lab's `k_lidar` controller.

## Speed

The SUPER yaml's `traj_opt/boundary/max_vel` is 1.5 m/s for the first flights and the lab yaml's `trajectory/max_speed` matches it (that one only warns). Raise them together.
