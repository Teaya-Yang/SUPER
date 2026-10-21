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
