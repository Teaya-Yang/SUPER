/**
* This file is part of SUPER
*
* Perception-aware cost map for the A* frontend. See the header for the idea.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*/

#include <path_search/perception_cost_map.h>
#include <utils/header/yaml_loader.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace path_search {

    PerceptionCostMap::Params::Params(const std::string &cfg_path, const std::string &ns) {
        yaml_loader::YamlLoader loader(cfg_path);
        loader.LoadParam(ns + "/enable", enable, false);
        loader.LoadParam(ns + "/plane_topic", plane_topic, std::string("/plane_normals"));
        loader.LoadParam(ns + "/weight", weight, 2.0);
        loader.LoadParam(ns + "/update_rate", update_rate, 2.0);
        loader.LoadParam(ns + "/grid_resolution", grid_resolution, 0.5);

        std::vector<double> half{10.0, 10.0, 2.0};
        loader.LoadParam(ns + "/half_extent", half, half);
        if (half.size() == 3) half_extent = Vec3f(half[0], half[1], half[2]);

        std::vector<double> range{0.1, 10.0};
        loader.LoadParam(ns + "/range", range, range);
        if (range.size() == 2) { range_min = range[0]; range_max = range[1]; }

        std::vector<double> elev{-27.0, 32.0};
        loader.LoadParam(ns + "/elevation_deg", elev, elev);
        if (elev.size() == 2) {
            elev_min_rad = elev[0] * M_PI / 180.0;
            elev_max_rad = elev[1] * M_PI / 180.0;
        }

        loader.LoadParam(ns + "/max_planes", max_planes, 200);
        loader.LoadParam(ns + "/sigma_r", sigma_r, 0.02);
        double sigma_theta_deg = 0.15;
        loader.LoadParam(ns + "/sigma_theta_deg", sigma_theta_deg, 0.15);
        sigma_theta_rad = sigma_theta_deg * M_PI / 180.0;
        loader.LoadParam(ns + "/eps_reg", eps_reg, 1e-6);
        loader.LoadParam(ns + "/logdet_ref", logdet_ref, 70.0);
        loader.LoadParam(ns + "/logdet_scale", logdet_scale, 10.0);
        loader.LoadParam(ns + "/cost_max", cost_max, 3.0);
        loader.LoadParam(ns + "/default_cost", default_cost, 0.0);
    }

    PerceptionCostMap::PerceptionCostMap(const Params &params) : prm_(params) {}

    void PerceptionCostMap::setPlanes(const vec_Vec3f &points, const vec_Vec3f &normals) {
        const size_t n = std::min(points.size(), normals.size());
        vec_Vec3f pts, nrm;
        // Same even subsampling rule as the hiperlab planner (MaybeSubsamplePlanes).
        if (prm_.max_planes > 0 && n > static_cast<size_t>(prm_.max_planes)) {
            const int m = prm_.max_planes;
            pts.reserve(m);
            nrm.reserve(m);
            if (m == 1) {
                pts.push_back(points[n / 2]);
                nrm.push_back(normals[n / 2]);
            } else {
                for (int i = 0; i < m; ++i) {
                    const size_t idx = (static_cast<size_t>(i) * (n - 1)) / (static_cast<size_t>(m) - 1);
                    pts.push_back(points[idx]);
                    nrm.push_back(normals[idx]);
                }
            }
        } else {
            pts.assign(points.begin(), points.begin() + n);
            nrm.assign(normals.begin(), normals.begin() + n);
        }
        std::lock_guard<std::mutex> lk(planes_mtx_);
        pts_.swap(pts);
        nrm_.swap(nrm);
    }

    int PerceptionCostMap::numPlanes() const {
        std::lock_guard<std::mutex> lk(planes_mtx_);
        return static_cast<int>(pts_.size());
    }

    double PerceptionCostMap::costFromLogDet(const double &logdet) const {
        const double c = (prm_.logdet_ref - logdet) / prm_.logdet_scale;
        return std::max(0.0, std::min(prm_.cost_max, c));
    }

    double PerceptionCostMap::planeVariance(const Vec3f &d, const Vec3f &n, const double &sigma_r,
                                            const double &sigma_theta_rad) {
        const double r = d.norm();
        if (r < 1e-9) return 0.0;
        const Vec3f r_hat = d / r;
        const double rho = std::hypot(d.x(), d.y());
        const double sin_el = std::max(-1.0, std::min(1.0, d.z() / r));
        Vec3f az_hat(0, 1, 0), el_hat(0, 0, 1);
        double cos_el = 1.0;
        if (rho > 1e-9) {
            const double cos_az = d.x() / rho, sin_az = d.y() / rho;
            cos_el = rho / r;
            az_hat = Vec3f(-sin_az, cos_az, 0.0);
            el_hat = Vec3f(-sin_el * cos_az, -sin_el * sin_az, cos_el);
        }
        const double n_r = n.dot(r_hat), n_a = n.dot(az_hat), n_e = n.dot(el_hat);
        const double sr2 = sigma_r * sigma_r, st2 = sigma_theta_rad * sigma_theta_rad;
        return sr2 * n_r * n_r + r * r * st2 * (cos_el * cos_el * n_a * n_a + n_e * n_e);
    }

    double PerceptionCostMap::logDetAt(const Vec3f &p, const vec_Vec3f &pts, const vec_Vec3f &nrm) const {
        // I = sum_k H_k^T H_k / sigma_k^2 with H_k = [n^T, (d x n)^T], d = q - p.
        // Noise, as in the lab code: spherical covariance diag(sr^2, st^2, st^2) pushed
        // through the spherical->Cartesian Jacobian J = [r_hat, r cos(el) az_hat, r el_hat],
        // so S = sr^2 r r^T + (r st)^2 (cos^2(el) az az^T + el el^T). Evaluated in a
        // level body frame; yaw does not change it because az_hat/el_hat only depend on
        // the z axis.
        Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();
        const size_t n = std::min(pts.size(), nrm.size());
        for (size_t k = 0; k < n; ++k) {
            const Vec3f d = pts[k] - p;
            const double r = d.norm();
            if (r < 1e-6 || r < prm_.range_min || r > prm_.range_max) continue;
            const double el = std::asin(std::max(-1.0, std::min(1.0, d.z() / r)));
            if (el < prm_.elev_min_rad || el > prm_.elev_max_rad) continue;

            const Vec3f &nv = nrm[k];
            const double var = planeVariance(d, nv, prm_.sigma_r, prm_.sigma_theta_rad);
            if (var < 1e-18) continue;

            Eigen::Matrix<double, 6, 1> h;
            h.head<3>() = nv;
            h.tail<3>() = d.cross(nv);   // -n^T [d]_x  ==  (d x n)^T
            info.noalias() += (h * h.transpose()) / var;
        }
        info.diagonal().array() += prm_.eps_reg;

        Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(info);
        if (llt.info() == Eigen::Success) {
            const Eigen::Matrix<double, 6, 6> L = llt.matrixL();
            double logdet = 0.0;
            for (int i = 0; i < 6; ++i) logdet += std::log(L(i, i));
            return 2.0 * logdet;
        }
        double det = info.determinant();
        if (det <= 0.0) det = std::pow(prm_.eps_reg, 6);
        return std::log(det);
    }

    bool PerceptionCostMap::update(const Vec3f &robot_pos) {
        std::unique_lock<std::mutex> busy(update_mtx_, std::try_to_lock);
        if (!busy.owns_lock()) return false;   // previous build still running

        vec_Vec3f pts, nrm;
        {
            std::lock_guard<std::mutex> lk(planes_mtx_);
            pts = pts_;
            nrm = nrm_;
        }
        if (pts.empty()) return false;

        const auto t0 = std::chrono::steady_clock::now();
        auto g = std::make_shared<Grid>();
        g->res = prm_.grid_resolution;
        // Snap the origin to the grid so cells do not jitter between updates.
        const Vec3f lo = robot_pos - prm_.half_extent;
        g->origin = (lo / g->res).array().floor().matrix() * g->res;
        g->dims = (2.0 * prm_.half_extent / g->res).array().ceil().cast<int>().matrix();
        g->dims = g->dims.cwiseMax(Eigen::Vector3i(1, 1, 1));
        const int N = g->size();
        g->cost.assign(N, 0.f);
        g->logdet.assign(N, 0.f);
        g->n_planes = static_cast<int>(pts.size());

        const int ny = g->dims.y(), nz = g->dims.z();
#pragma omp parallel for schedule(dynamic, 64)
        for (int idx = 0; idx < N; ++idx) {
            const int i = idx / (ny * nz);
            const int j = (idx / nz) % ny;
            const int k = idx % nz;
            const double L = logDetAt(g->centre(i, j, k), pts, nrm);
            g->logdet[idx] = static_cast<float>(L);
            g->cost[idx] = static_cast<float>(costFromLogDet(L));
        }

        g->build_time_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        g->stamp = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        {
            std::lock_guard<std::mutex> lk(grid_mtx_);
            grid_ = g;
        }
        return true;
    }

    std::shared_ptr<const PerceptionCostMap::Grid> PerceptionCostMap::snapshot() const {
        std::lock_guard<std::mutex> lk(grid_mtx_);
        return grid_;
    }
}
