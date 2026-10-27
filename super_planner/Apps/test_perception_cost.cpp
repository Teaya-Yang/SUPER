/**
* Standalone check of PerceptionCostMap (no ROS).
*
*   rosrun super_planner test_perception_cost
*
* Prints the log det and cost for a few synthetic plane layouts so the numbers
* behind logdet_ref are visible, checks the closed-form noise projection against
* the hiperlab J*S*J^T computation, and times a grid build.
*/

#include <path_search/perception_cost_map.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>

using path_search::PerceptionCostMap;
using path_search::Vec3f;
using path_search::vec_Vec3f;

namespace {

// Planes on the faces of an axis-aligned box of half size h around c, n_per_face each.
void boxPlanes(const Vec3f &c, double h, int n_per_face, vec_Vec3f &pts, vec_Vec3f &nrm, bool walls_x, bool walls_y,
               bool floor, bool ceil) {
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> u(-h, h);
    // Walls span +-h horizontally and 0..3 m in height; floor is z = 0, ceiling z = 3.
    auto add = [&](const Vec3f &n, int axis, double sign) {
        for (int i = 0; i < n_per_face; ++i) {
            Vec3f p(c.x() + u(rng), c.y() + u(rng), 1.5 + u(rng) * 1.5 / h);
            if (axis < 2) p(axis) = c(axis) + sign * h;
            else p.z() = sign < 0 ? 0.0 : 3.0;
            pts.push_back(p);
            nrm.push_back(n);
        }
    };
    if (walls_x) { add(Vec3f(1, 0, 0), 0, -1); add(Vec3f(1, 0, 0), 0, +1); }
    if (walls_y) { add(Vec3f(0, 1, 0), 1, -1); add(Vec3f(0, 1, 0), 1, +1); }
    if (floor) add(Vec3f(0, 0, 1), 2, -1);
    if (ceil) add(Vec3f(0, 0, 1), 2, +1);
}

// The lab's per-plane variance: n^T (R J S_sph J^T R^T) n with R = identity (level, yaw 0).
double labVariance(const Vec3f &d, const Vec3f &n, double sr, double st) {
    const double r = d.norm();
    const double az = std::atan2(d.y(), d.x());
    const double el = std::asin(d.z() / r);
    Eigen::Matrix3d J;
    J << std::cos(el) * std::cos(az), -r * std::cos(el) * std::sin(az), -r * std::sin(el) * std::cos(az),
            std::cos(el) * std::sin(az), r * std::cos(el) * std::cos(az), -r * std::sin(el) * std::sin(az),
            std::sin(el), 0.0, r * std::cos(el);
    Eigen::Matrix3d S = Eigen::Vector3d(sr * sr, st * st, st * st).asDiagonal();
    Eigen::Matrix3d Sc = J * S * J.transpose();
    return n.transpose() * Sc * n;
}

}  // namespace

int main() {
    PerceptionCostMap::Params prm;   // defaults, including the elevation band
    prm.enable = true;
    PerceptionCostMap pcm(prm);
    const Vec3f c(0, 0, 1.5);

    struct Case { const char *name; bool wx, wy, fl, ce; };
    const Case cases[] = {
            {"room: 4 walls + floor + ceiling", true, true, true, true},
            {"corridor along y: 2 walls (x) + floor", true, false, true, false},
            {"floor only", false, false, true, false},
            {"nothing visible", false, false, false, false},
    };
    std::printf("logdet_ref = %.1f, logdet_scale = %.1f, cost_max = %.1f, elevation band %.0f..%.0f deg\n\n",
                prm.logdet_ref, prm.logdet_scale, prm.cost_max, prm.elev_min_rad * 180 / M_PI,
                prm.elev_max_rad * 180 / M_PI);
    std::printf("%-46s %10s %8s\n", "layout (walls +-4 m, drone at 1.5 m, 50/face)", "logdet", "cost");
    for (const auto &cs : cases) {
        vec_Vec3f pts, nrm;
        boxPlanes(c, 4.0, 50, pts, nrm, cs.wx, cs.wy, cs.fl, cs.ce);
        const double L = pcm.logDetAt(c, pts, nrm);
        std::printf("%-46s %10.2f %8.2f\n", cs.name, L, pcm.costFromLogDet(L));
    }

    // Noise-projection equivalence against the lab's Jacobian form.
    {
        std::mt19937 rng(7);
        std::uniform_real_distribution<double> u(-1, 1);
        double max_rel = 0;
        for (int i = 0; i < 1000; ++i) {
            Vec3f d(u(rng) * 8, u(rng) * 8, u(rng) * 3);
            Vec3f n(u(rng), u(rng), u(rng));
            if (d.norm() < 0.5 || n.norm() < 1e-3) continue;
            n.normalize();
            const double mine = PerceptionCostMap::planeVariance(d, n, prm.sigma_r, prm.sigma_theta_rad);
            const double lab = labVariance(d, n, prm.sigma_r, prm.sigma_theta_rad);
            max_rel = std::max(max_rel, std::abs(mine - lab) / lab);
        }
        std::printf("\nnoise projection vs lab J*S*J^T: max relative difference %.2e (%s)\n", max_rel,
                    max_rel < 1e-9 ? "ok" : "MISMATCH");
    }

    // Grid build timing with a realistic plane count.
    {
        vec_Vec3f pts, nrm;
        boxPlanes(c, 6.0, 40, pts, nrm, true, true, true, false);   // 200 planes
        pcm.setPlanes(pts, nrm);
        const auto t0 = std::chrono::steady_clock::now();
        pcm.update(c);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        auto g = pcm.snapshot();
        int pen = 0;
        for (float v : g->cost) pen += v > 0.f;
        std::printf("grid %dx%dx%d (%.1f m) from %d planes: %.1f ms, %d/%d cells penalized\n",
                    g->dims.x(), g->dims.y(), g->dims.z(), g->res, g->n_planes, ms, pen, g->size());
        std::printf("cost at centre %.2f, cost far outside grid (default) %.2f\n",
                    PerceptionCostMap::cost(*g, c, prm.default_cost),
                    PerceptionCostMap::cost(*g, c + Vec3f(100, 0, 0), prm.default_cost));
    }
    return 0;
}
