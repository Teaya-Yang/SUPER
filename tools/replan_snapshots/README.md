# Replan snapshots

Compare SUPER's A* guide path with the optimized exploratory trajectory, one replan cycle at a time.

The planner already publishes everything needed on its RViz topics. These scripts record those topics, extract one snapshot per replan, and build a self-contained HTML viewer.

## Usage

Inside the container (`bash docker/noetic/super.sh shell`), with the planner running:

```bash
bash tools/replan_snapshots/record_replans.sh ~/ws_super/bags/run.bag 40
python3 tools/replan_snapshots/extract_replans.py ~/ws_super/bags/run.bag ~/ws_super/bags/run_replans.json
```

On the host or in the container:

```bash
python3 tools/replan_snapshots/make_viewer.py ~/ws_super/bags/run_replans.json ~/ws_super/bags/run_replans.html \
    --label "benchmark_dense" --bound 0.8
```

Open the HTML file in a browser. `--bound` is the run's `corridor_bound_dis`, drawn as a reference line. The page has a top-down view and a 3D view; the 3D view loads three.js from cdn.jsdelivr.net, so it needs internet access.

## What is shown

- **Guide path**, split into the part reused from the previously committed trajectory and the new A* part. A guide point counts as reused if it is within 5 cm of the previous committed trajectory.
- **Corridor**: the top-down outline of each polytope.
- **Optimized trajectory**: the new exploratory trajectory from this replan.
- **Committed backup trajectory**, and the flown path up to that cycle.
- **Deviation**: per cycle, the largest and mean distance from the optimized trajectory to the guide path.

Obstacles are map points from `/global_pc` above the ground, 0.3 to 4.5 m by default, stored as 0.2 m voxels. Change the band with `--zmin` and `--zmax` on the extractor.

The top-down view only draws obstacles within 0.3 m of the heights that cycle's paths fly at. Without that, obstacles the drone flies over or under look like collisions. The 3D view draws all of them.

## Limits

- Only works with the bundled simulator as is, because it uses `/global_pc` for obstacles. With FAST_LIO, record a map cloud topic instead and adapt the extractor.
- The exploratory trajectory and corridor markers are published only when something subscribes, so keep the recorder running for the whole flight.
