# SUPER study notes

Two goals:

1. Understand the code well enough to add a perception-aware cost or constraint.
2. Run SUPER on the lab stack: UnitySim + FAST_LIO + hiperlab low-level sim, instead of `perfect_drone_sim`.

## Status (2026-10-21)

- Docker setup works. See `docker/noetic/README.md`. Use `bash docker/noetic/super.sh shell` for a GUI-capable shell.
- Rendering in the container is CPU-only (llvmpipe). Installing `nvidia-container-toolkit` on the host, then running `super.sh rm && super.sh up`, switches it to the GPU.
- Code reading for goal 1 is done. Goal 2 has not been started.

## Next session: perception cost in the optimizer

Start here:

- `super_planner/src/traj_opt/exp_traj_optimizer_s4.cpp`
  - `costFunctional` (around line 251): the L-BFGS callback. Computes MINCO energy, calls `constraintsFunctional`, then propagates gradients to waypoints and times.
  - `constraintsFunctional` (around line 45): samples each segment and adds the soft penalties. This is where the new term goes.

### To do

- [ ] Decide where the term belongs.
  - Position/attitude (tilt) of the trajectory: the exploratory optimizer, `exp_traj_optimizer_s4.cpp`.
  - Heading only: the yaw optimizer, `yaw_traj_opt.cpp`. Yaw is fixed at 0 inside the exploratory optimizer's flatness map.
  - Backup trajectory too? Probably not. It is a separate optimizer, `backup_traj_optimizer_s4.cpp`.
- [ ] Write the term as f(p, v, a, j) at a sample point, or a function of attitude.
- [ ] Derive its gradient by hand: df/dp, df/dv, df/da, df/dj. If it uses attitude, derive df/dq.
- [ ] Make it smooth (continuous first derivative). Use `gcopter::smoothedL1` for hinge-style penalties.
- [ ] Choose the data it needs and how it gets into the optimizer.
  - Raw ROG-Map occupancy has zero gradient almost everywhere, so it is not usable directly.
  - Options: ESDF (disabled in demo config, `rog_map/esdf/enable: 0`), or an analytic field such as Gaussians around features or a smooth FOV cone.
  - Precompute before the solve. Keep the per-sample cost cheap.
- [ ] Plumbing:
  - Add the data to `OptimizationVariables` and pass it into `constraintsFunctional`.
  - Add an 8th weight to `penaltyWeights` (resized to 7 around line 779) and a YAML parameter.
  - Extend `penalty_log` (resized to 8 around line 621) and the penalty printout (around line 704).
- [ ] Write a finite-difference gradient check: perturb p, v, a, j by about 1e-6 and compare against the analytic gradient. Do this before any flight test.
- [ ] Test with `receding_dis: 0` first, so the warm start doesn't hide the new term's effect.

### Template: how the corridor term gets map data (2026-10-26)

No optimizer term reads ROG-Map directly. The map reaches the optimizer only through the corridor, and that term is the closest template for a plane-normal cost:

1. `super_core/corridor_generator.cpp` queries ROG-Map (`boxSearch`, around lines 212 and 308), and CIRI builds convex polytopes.
2. Each polytope is stored as half-space rows `[n_x, n_y, n_z, d]`, with n the outward plane normal.
3. `ExpTrajOpt::processCorridor` (around line 354) puts them into `opt_vars.hPolytopes`, plus `hPolyIdx` (segment to polytope).
4. In `constraintsFunctional` (around lines 122 to 135), each sample penalizes `n·p + d > 0` with `smoothedL1` and adds `weight · penalty' · n` to `gradPos`.

Copy how `hPolytopes` and `hPolyIdx` travel into `OptimizationVariables` for the new data.

Notes for the plane-normal term:

- [ ] LIO planes are not in ROG-Map, which is occupancy only. Publish them from FAST_LIO, subscribe in `include/ros_interface/ros1/ros1_interface.hpp`, store them in the planner, and pass them into the optimizer the same way as the corridor.
- [ ] Visibility of a plane from a sample must be soft, for example weighted by distance and incidence angle. Hard tests break the gradient.
- [ ] Depends on position and tilt. Heading matters less with the Mid-360's full horizontal FOV. Tilt goes through the quaternion gradient slot in `flatMap.backward`.
- [ ] Reduce the plane set before the solve. The corridor term loops over about 10 to 20 faces per sample, and hundreds of planes per sample would be slow.
- If a map query inside the loop is ever needed: ROG-Map's ESDF (`rog_map/include/rog_map/esdf_map.h`) has `evaluateEDT` and `evaluateFirstGrad` with trilinear interpolation. Unused by SUPER and disabled in the demo config.

### How the existing terms work (for reference)

At each sample in `constraintsFunctional`, each term:

1. Adds its scalar cost to `tmp_cost`.
2. Adds its partial derivatives to `gradPos`, `gradVel`, `gradAcc`, `gradJer`.
3. Logs its worst violation in `max_pena`.

The code after that (around lines 225 to 237) turns these into gradients on coefficients and segment times, with trapezoid weights. Explicit time dependence, such as a moving target, is not covered and would need its own time-gradient term.

Attitude-dependent terms:

- `flatMap.forward` (around line 196) gives thrust, quaternion and body rates from v, a, j.
- `flatMap.backward` (around line 215) takes a quaternion gradient, currently `Vec4f(0,0,0,0)`. Put df/dq there.
- The flatness branch only runs when both `weightOmg` and `weightAccThr` are positive (around line 191). A new attitude term needs its own call, or a changed condition.

### Watch out

- Everything is a soft penalty. Only segment endpoints are hard-constrained to the corridor. A strong perception weight can pull samples out of the corridor, so keep it well below `penna_pos`.
- A wrong gradient usually doesn't crash. L-BFGS stops early on a line-search failure and returns a poor trajectory.

## Alternative: perception cost in A* (2026-10-26, current plan)

Reason: visibility is discontinuous, so it is hard to put in the gradient-based optimizer. A* needs no gradient or smoothness. Hard visibility tests and raycasts are fine if cached.

How the search works (`super_planner/src/super_core/astar.cpp`, `pointToPointPathSearch` around line 233):

- Grid on the inflated map (0.2 m), local box 500 x 500 x 100 cells around the start, 26-connected.
- Edge cost is step length only (around line 486). Heuristic is Euclidean times 1 + 1e-5 (`heu_type: 2`, around line 109).
- Stops at the goal, or when the popped node's `g` exceeds the horizon (around line 392). Timeout is 0.1 s.
- Nodes store only grid index, scores and parent. No velocity or attitude.

To do:

- [ ] Edge cost becomes `g += step * (1 + w * c(pos))` at around line 486.
- [ ] Add a separate path-length field to `GridNode` and use it in the horizon check (around line 392). Otherwise the search stops early in poorly observed areas.
- [ ] Keep c(pos) >= 0, a penalty for poor observability. The Euclidean heuristic then stays a lower bound. A negative reward breaks A*.
- [ ] Compute c(pos) lazily per cell and cache it on `GridNode`, or precompute a grid over the search box. A raycast per neighbour without a cache is too slow.
- [ ] Heading can only come from travel direction (dx, dy, dz) or the parent cell. Position alone may be enough with the Mid-360.
- [ ] Fix the open-set update: at around line 500, an improved node is not re-pushed (`std::priority_queue` has no decrease-key). Re-push it and skip stale entries when popped. This matters more once costs vary across cells.
- Known harmless bug: `inf` in `include/path_search/astar.h` (line 40) is `1 >> 20`, which is 0.

How much of the A* shape survives:

- Path is cut into seed segments of at most `corridor_line_max_length` (1.2 m click). Each polytope is clipped to its segment's bounding box plus `corridor_bound_dis` (1 m click, 3 m high-speed).
- The optimizer can move about `corridor_bound_dis` sideways and will cut corners. Detours larger than that survive. Lower `corridor_bound_dis` to make the trajectory follow A* more closely.
- In hot replans, A* only extends past the reused `receding_dis` of the old trajectory. Test with `receding_dis: 0`.

## A* vs optimizer, measured (2026-10-26)

Tool: `tools/replan_snapshots/` (see its README). Viewer for the dense benchmark: `~/ws_super/bags/dense_replans.html`.

Dense benchmark (`static_dense.yaml`: `corridor_bound_dis` 0.8, `receding_dis` 1.0, horizon 7 m), 510 replans, 477 with an optimized trajectory:

| Measure | Value |
|---|---|
| Max distance from optimized traj to guide path, median | 0.40 m |
| Same, 90th percentile | 0.62 m |
| Cycles exceeding the 0.8 m corridor bound | 4 |
| Mean distance, median | 0.17 m |
| Guide path reused / new A*, median length | 1.5 m / 5.5 m |

So the optimized trajectory stays within about half a metre of the A* path, and A* supplies about three quarters of each guide path. A perception term in A* should carry through, as long as its detours are larger than about 0.5 m.

Open issue: in the high-speed benchmark, about 880 plan-from-rest attempts failed in the exploratory optimizer ("Omg or thr or Pos violation") before the drone got going. Not investigated. CPU contention from software rendering may play a part.

## Perception-aware A* cost: implementation (2026-10-26)

Data flow:

```
/plane_normals (visualization_msgs/Marker LINE_LIST, from FAST_LIO)
  -> FsmRos1::planeNormalsCallback            (include/ros_interface/ros1/fsm_ros1.hpp)
  -> PerceptionCostMap::setPlanes             (include/path_search/perception_cost_map.h, src/super_core/perception_cost_map.cpp)
FsmRos1::perceptionTimerCallback (astar/perception/update_rate Hz)
  -> PerceptionCostMap::update(robot_pos)     builds a coarse grid around the drone, swaps it in atomically
  -> Ros1Interface::vizPerceptionCostGrid     one horizontal layer at the drone height:
                                              /fsm_node/visualization/perception_cost  (intensity = cost)
                                              /fsm_node/visualization/perception_logdet (intensity = raw log det)
Astar::pointToPointPathSearch                 takes one grid snapshot per search; edge cost = step * (1 + weight * cell_cost)
```

Per cell: `I = sum_k H_k^T H_k / sigma_k^2`, `H_k = [n^T, (d x n)^T]`, `d = plane point - cell centre`, planes within range and the level-frame elevation band, `logdet = log det(I + eps I6)`, `cost = clamp((logdet_ref - logdet) / logdet_scale, 0, cost_max)`. Same information matrix as the lab's `GetLidarAwareTrajCost`; the noise projection is written in closed form (`sigma_r^2 (n.r)^2 + (r sigma_theta)^2 (1 - (n.r)^2)`), which equals the lab's `J S J^T` form (checked in `Apps/test_perception_cost.cpp`).

A* changes (`src/super_core/astar.cpp`): `GridNode::path_len` is pure length and drives the horizon check; `distance_score` is the weighted g; improved open-set nodes are re-pushed and stale pops skipped; `inf` fixed; search time and expansions are logged by `PathSearch` when `super_planner/print_log` is on.

Config: `astar/perception/*` in the YAML (see `config/click_smooth_ros1.yaml`, disabled by default). `config/click_perception_ros1.yaml` enables it with `receding_dis: 0` and `print_log: true`.

Testing in the bundled sim:

```bash
roslaunch mission_planner click_perception_demo.launch featureless_box:="[-10, 2, 0, 0, 12, 4]"
rosrun super_planner test_perception_cost      # synthetic layouts, noise check, grid timing
```

`scripts/fake_plane_normals.py` fits planes to `/global_pc` by local PCA and publishes the ones a level Mid-360 at the drone would see. `featureless_box` drops planes inside a box to emulate a textureless region.

Tuning `logdet_ref`: look at `visualization/perception_logdet` during a normal flight. Typical values with ~200 planes are around 70 to 85; a corridor that leaves one translation direction unobserved drops by roughly 25; floor-only drops to about 0; no planes gives about -83. Set `logdet_ref` just below the normal-flight value so only weak regions get a cost.

Integration with FAST_LIO: planes are in `camera_init`, the same frame as `/Odometry_imu`, so no transform is needed as long as rog_map's odom topic is also FAST_LIO's.

### Sim A/B result (2026-10-26)

Click demo, start (0,0,1.5) facing +y, goal (0,15,1.5), `click_perception_ros1.yaml` (weight 3, logdet_ref 80, scale 5, receding_dis 0). Mean x of the guide path and the flown path for y in [3,12]:

| Trial | Guide path x | Flown x | A* per search |
|---|---|---|---|
| perception off | -0.90 m | -1.09 m | ~1.2 ms, ~1.6k expansions |
| featureless half-space x < -0.5 | +3.66 m | +4.10 m | 10.6 ms, 11.8k expansions |
| featureless half-space x > 0.5 | -5.01 m | -5.01 m | 9.0 ms, 10.0k expansions |

Two lessons:

- Log det is insensitive to plane count and only drops sharply on rank deficiency. A 12 m box barely moved the path (-0.74 vs -0.94); the half-space box worked. In the pillar forest nothing is ever rank deficient, so `logdet_ref` had to sit just under the normal value (82) for density differences to matter. On the real stack look at `visualization/perception_logdet` first.
- The cost weakens the Euclidean heuristic (edges cost up to (1 + weight * cost_max) times their length), so A* expands about 7x more nodes. Still ~10 ms here. If it grows, scale the heuristic by the smallest edge multiplier in the grid or use weighted A*.

Trial scripts: `~/ws_super/bags/run_trial.sh`, `~/ws_super/bags/analyze.py` (not in the repo).

## Later: integration with the lab stack

- [ ] Planner interface to match:

  | Direction | Topic | Type | YAML key |
  |---|---|---|---|
  | in | `/cloud_registered` | PointCloud2, world frame | `rog_map/ros_callback/cloud_topic` |
  | in | `/lidar_slam/odom` | Odometry | `rog_map/ros_callback/odom_topic` |
  | in | `/goal` | PoseStamped | `fsm/click_goal_topic` |
  | out | `/planning/pos_cmd` | `quadrotor_msgs/PositionCommand` (p, v, a, j, yaw, yaw rate) | `fsm/cmd_topic` |

- [ ] First test without merging workspaces. Run Unity, FAST_LIO and hiperlab in `ws-palio`, and `fsm_node` in `ws-super`. Both use host networking and the same ROS master.
- [ ] Copy `super_planner/config/click_smooth_ros1.yaml` to a new config. Point `odom_topic` at the FAST_LIO odometry. Check `rog_map/fix_map_origin`, `fsm/click_height` and frame conventions.
- [ ] In `GeneralCode/ROS/hiperlab_rostools`, find the message the low-level controller tracks. Decide whether it takes a full p/v/a/yaw sample or just a setpoint.
- [ ] Write an adapter node from `PositionCommand` to that message, plus a launch file for `fsm_node`, the new config and the adapter.

## Concepts worked out so far

- **Cold vs hot start** (`generateExpTraj`, around line 405 of `super_core/super_planner.cpp`). With no previous trajectory, the planner plans from rest. Otherwise it reuses the old trajectory for three things:
  1. The start state, `replan_forward_dt` (0.1 s) ahead on the committed trajectory, so the splice is continuous while the solve runs.
  2. A warm-start guide path. `receding_dis` metres of the old trajectory are kept, which avoids flipping between local minima. Set it to 0 to disable.
  3. Early exits when the old plan is still fine.
- **`NO_NEED` when past the end of the old trajectory.** The command sampler clamps to the trajectory's final state, so the drone hovers there. The command timer (`fsm_ros1.hpp`, around line 239) detects the finished trajectory and switches to `GENERATE_TRAJ` (plan from rest) or `WAIT_GOAL`. The replan step just leaves it alone.
- **Commented-out goal projection** (around line 615 of `super_planner.cpp`). Dead code with two bugs. A* now clips far goals to the local map and stops at the horizon (`astar.cpp`, `pointToPointPathSearch`). The FSM moves the goal out of obstacles before each replan.
- **`+ 0.01` on `t0`** (around line 990, backup trajectory). `t0` is "now" on the new trajectory's timeline, and it's the lower bound on the backup switch time. The 10 ms margin keeps the branch point ahead of the drone while the backup solve and commit finish. It's a hardcoded guess and may be too small on slow onboard computers.

## Optional tools

- `read_replan_log` and `traj_opt_tuning` replay recorded replans from `super_planner/log/replan_logs/*.bin`. They are commented out in `super_planner/CMakeLists.txt` (around lines 84 to 101). Uncomment and rebuild to use them. They are useful for testing a new cost term offline.
