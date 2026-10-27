/**
* This file is part of SUPER
*
* Perception-aware cost map for the A* frontend.
*
* Builds a coarse grid of "how well can LIO localize here" scores around the
* robot from the plane features FAST_LIO publishes on /plane_normals. The score
* per cell is the negative log-determinant of the 6x6 pose Fisher information
* of point-to-plane residuals, the same quantity the hiperlab lidar-aware
* planner uses (DepthImagePlanner::GetLidarAwareTrajCost), evaluated at the
* cell centre with a level body frame.
*
* Thread model: setPlanes() is called from a ROS subscriber, update() from a
* ROS timer, and A* reads a snapshot() of the finished grid. The grid is
* immutable once published, so A* never needs a lock inside its loop.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*/

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "utils/header/type_utils.hpp"

namespace path_search {
    using super_utils::Vec3f;
    using super_utils::vec_Vec3f;

    class PerceptionCostMap {
    public:
        struct Params {
            bool enable{false};
            std::string plane_topic{"/plane_normals"};
            double weight{2.0};            // A*: edge cost = step * (1 + weight * cost)
            double update_rate{2.0};       // Hz, grid rebuild rate
            double grid_resolution{0.5};   // m
            Vec3f half_extent{10.0, 10.0, 2.0};  // m, grid half size around the robot
            double range_min{0.1}, range_max{10.0};          // m, sensor range
            // Elevation band in a level world frame. The Mid-360 covers -7..52 deg in its
            // own frame; the lab mounts it pitched 20 deg down, giving -27..32 deg ahead.
            double elev_min_rad{-27.0 * M_PI / 180.0};
            double elev_max_rad{32.0 * M_PI / 180.0};
            int max_planes{200};           // evenly subsample the scan to this many planes
            double sigma_r{0.02};          // m, range noise
            double sigma_theta_rad{0.15 * M_PI / 180.0};     // angular noise
            double eps_reg{1e-6};          // regularisation added to the information matrix
            double logdet_ref{70.0};       // cost is 0 at or above this log det
            double logdet_scale{10.0};     // cost = (logdet_ref - logdet) / logdet_scale
            double cost_max{3.0};          // clamp
            double default_cost{0.0};      // outside the grid, or before the first update

            Params() = default;

            // Loads "<ns>/..." from the planner YAML; missing keys keep the defaults above.
            explicit Params(const std::string &cfg_path, const std::string &ns = "astar/perception");
        };

        struct Grid {
            Vec3f origin{0, 0, 0};         // min corner of cell (0,0,0)
            double res{0.5};
            Eigen::Vector3i dims{0, 0, 0};
            std::vector<float> cost;       // clamped, non-negative
            std::vector<float> logdet;     // raw log det, for tuning logdet_ref
            int n_planes{0};               // planes used for this grid
            double build_time_ms{0.0};
            double stamp{0.0};             // wall time of the build

            inline int size() const { return dims.x() * dims.y() * dims.z(); }

            // Flat index of the cell containing p, or -1 when outside the grid.
            inline int indexOf(const Vec3f &p) const {
                const int i = static_cast<int>(std::floor((p.x() - origin.x()) / res));
                const int j = static_cast<int>(std::floor((p.y() - origin.y()) / res));
                const int k = static_cast<int>(std::floor((p.z() - origin.z()) / res));
                if (i < 0 || j < 0 || k < 0 || i >= dims.x() || j >= dims.y() || k >= dims.z()) return -1;
                return (i * dims.y() + j) * dims.z() + k;
            }

            inline Vec3f centre(const int &i, const int &j, const int &k) const {
                return origin + res * Vec3f(i + 0.5, j + 0.5, k + 0.5);
            }
        };

        typedef std::shared_ptr<PerceptionCostMap> Ptr;

        explicit PerceptionCostMap(const Params &params);

        const Params &params() const { return prm_; }

        // Replace the stored plane set (latest LIO scan). Points and normals are in the
        // same world frame as the odometry fed to ROG-Map. Called from a subscriber.
        void setPlanes(const vec_Vec3f &points, const vec_Vec3f &normals);

        int numPlanes() const;

        // Rebuild the grid around robot_pos from the current planes and publish it
        // atomically. Returns false when there are no planes yet. Called from a timer.
        bool update(const Vec3f &robot_pos);

        // Latest finished grid, or nullptr before the first update.
        std::shared_ptr<const Grid> snapshot() const;

        // Nearest-cell cost lookup; default_cost outside the grid.
        static double cost(const Grid &g, const Vec3f &p, const double &default_cost) {
            const int idx = g.indexOf(p);
            return idx < 0 ? default_cost : static_cast<double>(g.cost[idx]);
        }

        // log det of the regularised 6x6 Fisher information at p from the given planes.
        // Public so it can be unit-tested without ROS.
        double logDetAt(const Vec3f &p, const vec_Vec3f &pts, const vec_Vec3f &nrm) const;

        double costFromLogDet(const double &logdet) const;

        // Point-to-plane residual variance for a plane with normal n seen along d (world,
        // level body frame): the lab's n^T (J S_sph J^T) n with J = [r_hat, r cos(el) az_hat, r el_hat].
        static double planeVariance(const Vec3f &d, const Vec3f &n, const double &sigma_r,
                                    const double &sigma_theta_rad);

    private:
        Params prm_;

        mutable std::mutex planes_mtx_;
        vec_Vec3f pts_, nrm_;

        mutable std::mutex grid_mtx_;
        std::shared_ptr<const Grid> grid_;

        std::mutex update_mtx_;   // keeps two timer threads from building at once
    };
}
