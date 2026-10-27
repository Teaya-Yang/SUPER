#!/usr/bin/env python3
"""Extract per-replan snapshots (guide path, corridor, optimized trajectory)
from a bag of SUPER's RViz topics, for comparing the A* frontend with the
optimized exploratory trajectory.

Run inside the ROS container (needs the rosbag Python API):
    python3 extract_replans.py run.bag run_replans.json [--zmin 0.3] [--zmax 4.5]

--zmin/--zmax select the map points kept as obstacles. The default lower
bound leaves out the ground plane.

Record the bag with record_replans.sh. Obstacles are stored as 3D voxels, corridors as the vertices of each 3D polytope.
"""
import argparse
import json
import math

import rosbag
from sensor_msgs import point_cloud2

T_PATH = "/fsm_node/visualization/frontend_path"
T_SFC = "/fsm_node/visualization/exp_sfc"
T_EXP = "/fsm_node/visualization/exp_traj"
T_CMT = "/fsm_node/visualization/committed_traj"
T_ODOM = "/lidar_slam/odom"
T_MAP = "/global_pc"

DELETEALL = 3
REUSE_TOL = 0.05     # m, guide point counts as reused if this close to the previous committed traj
OBST_CELL = 0.2      # m, obstacle voxel size
ODOM_DT = 0.1        # s, odometry downsampling


def r3(v):
    return round(v, 3)


def seg_dist(p, a, b):
    ax, ay, az = a
    dx, dy, dz = b[0] - ax, b[1] - ay, b[2] - az
    L2 = dx * dx + dy * dy + dz * dz
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy + (p[2] - az) * dz) / L2))
    qx, qy, qz = ax + t * dx, ay + t * dy, az + t * dz
    return math.sqrt((p[0] - qx) ** 2 + (p[1] - qy) ** 2 + (p[2] - qz) ** 2)


def poly_dist(p, line):
    if not line:
        return float("inf")
    if len(line) == 1:
        return math.dist(p, line[0])
    return min(seg_dist(p, line[i], line[i + 1]) for i in range(len(line) - 1))


def arrows_to_polyline(markers, ns):
    """Trajectory markers are 2-point ARROW segments, in order."""
    out = []
    for m in markers:
        if m.ns != ns or len(m.points) < 2:
            continue
        a, b = m.points[0], m.points[1]
        if not out:
            out.append((a.x, a.y, a.z))
        out.append((b.x, b.y, b.z))
    return out


def main(bag_path, out_path, zmin, zmax):
    bag = rosbag.Bag(bag_path)
    t0 = bag.get_start_time()

    obstacles = set()
    odom = []
    cycles = []
    cur = None
    prev_committed = []
    last_odom_t = -1e9

    for topic, msg, t in bag.read_messages(topics=[T_PATH, T_SFC, T_EXP, T_CMT, T_ODOM, T_MAP]):
        ts = t.to_sec() - t0
        if topic == T_MAP:
            for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                if zmin <= z <= zmax:
                    obstacles.add((round(x / OBST_CELL), round(y / OBST_CELL), round(z / OBST_CELL)))
            continue
        if topic == T_ODOM:
            if ts - last_odom_t >= ODOM_DT:
                p = msg.pose.pose.position
                odom.append([r3(ts), r3(p.x), r3(p.y), r3(p.z)])
                last_odom_t = ts
            continue
        if not msg.markers or all(m.action == DELETEALL for m in msg.markers):
            continue

        if topic == T_PATH:
            guide = [(m.pose.position.x, m.pose.position.y, m.pose.position.z)
                     for m in msg.markers if m.ns == "guide_path"]
            # Reused prefix: leading guide points lying on the previously committed trajectory.
            n_reused = 0
            for p in guide:
                if poly_dist(p, prev_committed) <= REUSE_TOL:
                    n_reused += 1
                else:
                    break
            cur = {"t": r3(ts), "guide": [[r3(c) for c in p] for p in guide], "n_reused": n_reused,
                   "sfc": [], "exp": [], "committed": [], "backup": []}
            cycles.append(cur)
        elif cur is None:
            continue
        elif topic == T_SFC:
            polys = []
            for m in msg.markers:
                if m.ns == "exp_sfc mesh" and m.points:
                    # The mesh is the polytope's triangulated hull; keep its unique vertices.
                    verts = sorted(set((round(p.x, 2), round(p.y, 2), round(p.z, 2)) for p in m.points))
                    polys.append([list(v) for v in verts])
            cur["sfc"] = polys
        elif topic == T_EXP:
            cur["exp"] = [[r3(c) for c in p] for p in arrows_to_polyline(msg.markers, "exp_traj")]
        elif topic == T_CMT:
            exp = arrows_to_polyline(msg.markers, "committed_exp")
            bak = arrows_to_polyline(msg.markers, "committed_backup")
            cur["committed"] = [[r3(c) for c in p] for p in exp]
            cur["backup"] = [[r3(c) for c in p] for p in bak]
            prev_committed = exp + bak

    # How far the optimized trajectory strays from the guide path, per cycle.
    for c in cycles:
        if c["exp"] and len(c["guide"]) >= 2:
            d = [poly_dist(p, c["guide"]) for p in c["exp"]]
            c["dev_max"] = r3(max(d))
            c["dev_mean"] = r3(sum(d) / len(d))
        else:
            c["dev_max"] = c["dev_mean"] = None

    out = {
        "bag": bag_path,
        "reuse_tol": REUSE_TOL,
        "obstacle_cell": OBST_CELL,
        "obstacle_z": [zmin, zmax],
        # Flat list of voxel indices i, j, k; multiply by obstacle_cell for metres.
        "voxels": [v for ijk in sorted(obstacles) for v in ijk],
        "odom": odom,
        "cycles": cycles,
    }
    with open(out_path, "w") as f:
        json.dump(out, f, separators=(",", ":"))

    ok = [c for c in cycles if c["dev_max"] is not None]
    print(f"{len(cycles)} replan cycles, {len(ok)} with an optimized trajectory, "
          f"{len(obstacles)} obstacle voxels -> {out_path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("out_json")
    ap.add_argument("--zmin", type=float, default=0.3, help="lowest map point drawn as obstacle [m]")
    ap.add_argument("--zmax", type=float, default=4.5, help="highest map point drawn as obstacle [m]")
    a = ap.parse_args()
    main(a.bag, a.out_json, a.zmin, a.zmax)
