#!/usr/bin/env python3
"""Fake /plane_normals publisher for the bundled SUPER simulator.

FAST_LIO publishes the planes it matched in the last scan as a LINE_LIST marker
(p0 -> p0 + 0.3 n, world frame). The bundled simulator has no LIO, so this node
builds a plane set once from the simulator's global map (/global_pc) by local
PCA, and at each tick publishes the planes a Mid-360 at the drone's position
would see: within range and the vertical FOV, assuming a level body.

Knobs (private params):
  ~featureless_box [xmin, ymin, zmin, xmax, ymax, zmax]  drop planes inside it
                                                         (emulates a textureless region)
  ~dropout          fraction of visible planes dropped at random each tick
  ~max_planes       even subsample of the visible set
"""
import numpy as np
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker


def cloud_to_xyz(msg):
    """PointCloud2 -> (N,3) float64 without the slow Python per-point reader."""
    offs = {f.name: (f.offset, f.datatype) for f in msg.fields}
    dt = {7: np.float32, 8: np.float64}
    names, formats, offsets = [], [], []
    for n in ("x", "y", "z"):
        o, t = offs[n]
        names.append(n)
        formats.append(dt[t])
        offsets.append(o)
    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})
    arr = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def fit_planes(xyz, voxel, min_pts, planarity):
    """Local PCA per voxel over its 3x3x3 neighbourhood. Returns (points, normals)."""
    lo = xyz.min(axis=0) - voxel
    idx = np.floor((xyz - lo) / voxel).astype(np.int64)
    dims = idx.max(axis=0) + 2
    flat = (idx[:, 0] * dims[1] + idx[:, 1]) * dims[2] + idx[:, 2]
    ncell = int(np.prod(dims))

    cnt = np.bincount(flat, minlength=ncell).astype(np.float64)
    s = np.stack([np.bincount(flat, weights=xyz[:, i], minlength=ncell) for i in range(3)], axis=1)
    pairs = [(0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2)]
    q = np.stack([np.bincount(flat, weights=xyz[:, a] * xyz[:, b], minlength=ncell) for a, b in pairs], axis=1)

    cnt3 = cnt.reshape(dims)
    s3 = s.reshape(tuple(dims) + (3,))
    q3 = q.reshape(tuple(dims) + (6,))
    # Sum over the 27-neighbourhood with shifted slices.
    pad = lambda a: np.pad(a, [(1, 1), (1, 1), (1, 1)] + [(0, 0)] * (a.ndim - 3))
    cp, sp, qp = pad(cnt3), pad(s3), pad(q3)
    C = np.zeros_like(cnt3)
    S = np.zeros_like(s3)
    Q = np.zeros_like(q3)
    for di in range(3):
        for dj in range(3):
            for dk in range(3):
                sl = (slice(di, di + dims[0]), slice(dj, dj + dims[1]), slice(dk, dk + dims[2]))
                C += cp[sl]
                S += sp[sl]
                Q += qp[sl]

    occ = cnt3 > 0
    good = occ & (C >= min_pts)
    C = C[good]
    S = S[good]
    Q = Q[good]
    mean = S / C[:, None]
    cov = np.empty((len(C), 3, 3))
    k = 0
    for a, b in pairs:
        v = Q[:, k] / C - mean[:, a] * mean[:, b]
        cov[:, a, b] = v
        cov[:, b, a] = v
        k += 1
    w, v = np.linalg.eigh(cov)                 # ascending eigenvalues
    w = np.clip(w, 0, None)
    flatness = w[:, 0] / np.maximum(w.sum(axis=1), 1e-12)
    keep = flatness < planarity
    normals = v[keep][:, :, 0]
    # FAST_LIO's sign is arbitrary; make it deterministic here (point +z-ish / +x-ish).
    sign = np.where(normals[:, 2] < -1e-6, -1.0, 1.0)
    normals = normals * sign[:, None]
    return mean[keep], normals


class FakePlaneNormals:
    def __init__(self):
        self.voxel = rospy.get_param("~voxel", 0.3)
        self.min_pts = int(rospy.get_param("~min_pts", 8))
        self.planarity = rospy.get_param("~planarity", 0.05)
        self.range = rospy.get_param("~range", [0.1, 10.0])
        elev = rospy.get_param("~elevation_deg", [-27.0, 32.0])
        self.elev = (np.deg2rad(elev[0]), np.deg2rad(elev[1]))
        self.max_planes = int(rospy.get_param("~max_planes", 300))
        self.dropout = float(rospy.get_param("~dropout", 0.0))
        self.box = rospy.get_param("~featureless_box", [])
        self.rate = rospy.get_param("~rate", 10.0)
        self.frame = rospy.get_param("~frame_id", "world")
        self.normal_scale = rospy.get_param("~normal_scale", 0.3)
        self.rng = np.random.default_rng(0)

        self.pts = None
        self.nrm = None
        self.pos = None
        self.pub = rospy.Publisher(rospy.get_param("~topic", "/plane_normals"), Marker, queue_size=1)
        self.box_pub = rospy.Publisher("~featureless_box_marker", Marker, queue_size=1, latch=True)
        self.map_sub = rospy.Subscriber(rospy.get_param("~map_topic", "/global_pc"), PointCloud2, self.on_map, queue_size=1)
        self.odom_sub = rospy.Subscriber(rospy.get_param("~odom_topic", "/lidar_slam/odom"), Odometry, self.on_odom, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.tick)
        self.publish_box()
        rospy.loginfo("[fake_plane_normals] waiting for the global map ...")

    def on_map(self, msg):
        if self.pts is not None:
            return
        xyz = cloud_to_xyz(msg)
        t0 = rospy.Time.now()
        pts, nrm = fit_planes(xyz, self.voxel, self.min_pts, self.planarity)
        self.pts, self.nrm = pts, nrm
        rospy.loginfo("[fake_plane_normals] %d map points -> %d planes in %.1f s",
                      len(xyz), len(pts), (rospy.Time.now() - t0).to_sec())
        self.map_sub.unregister()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pos = np.array([p.x, p.y, p.z])

    def publish_box(self):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        m.ns = "featureless_box"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD if len(self.box) == 6 else Marker.DELETE
        if len(self.box) == 6:
            b = np.array(self.box, dtype=float)
            c = (b[:3] + b[3:]) / 2
            s = np.maximum(b[3:] - b[:3], 0.05)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = c
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = s
            m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.3, 0.1, 0.15
        self.box_pub.publish(m)

    def tick(self, _):
        if self.pts is None or self.pos is None:
            return
        d = self.pts - self.pos
        r = np.linalg.norm(d, axis=1)
        with np.errstate(invalid="ignore", divide="ignore"):
            el = np.arcsin(np.clip(d[:, 2] / np.maximum(r, 1e-9), -1, 1))
        vis = (r >= self.range[0]) & (r <= self.range[1]) & (el >= self.elev[0]) & (el <= self.elev[1])
        if len(self.box) == 6:
            b = self.box
            inside = ((self.pts[:, 0] >= b[0]) & (self.pts[:, 0] <= b[3]) &
                      (self.pts[:, 1] >= b[1]) & (self.pts[:, 1] <= b[4]) &
                      (self.pts[:, 2] >= b[2]) & (self.pts[:, 2] <= b[5]))
            vis &= ~inside
        ids = np.nonzero(vis)[0]
        if self.dropout > 0 and len(ids):
            ids = ids[self.rng.random(len(ids)) >= self.dropout]
        if self.max_planes > 0 and len(ids) > self.max_planes:
            # even subsample over the (spatially unordered) index list
            sel = (np.arange(self.max_planes) * (len(ids) - 1)) // (self.max_planes - 1)
            ids = ids[sel]

        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        m.ns = "plane_normals"
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03
        m.color.r, m.color.a = 1.0, 1.0
        for i in ids:
            p0 = self.pts[i]
            p1 = p0 + self.normal_scale * self.nrm[i]
            m.points.append(Point(*p0))
            m.points.append(Point(*p1))
        self.pub.publish(m)


if __name__ == "__main__":
    rospy.init_node("fake_plane_normals")
    FakePlaneNormals()
    rospy.spin()
