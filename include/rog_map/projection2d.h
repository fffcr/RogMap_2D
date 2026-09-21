#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <vector>
#include <Eigen/Core>

namespace rog_map {
    //调试信息
    struct MinZResult {
    double distance = 0.0;   
    bool   valid    = false; 
    bool   blocked  = false; 
    int    n_sampled = 0;
    int    n_blocked = 0; 
};

    struct MinZConfig {
    bool enable = false;
    // double scan_z_min_rel = -0.05; // 扫描带下沿，相对 odom 的 z（米）
    // double scan_z_max_rel =  0.60; // 扫描带上沿，相对 odom 的 z（米）
    double max_distance   =  6.0;  // 正距离截断上限
    double min_distance   = -3.0;  //负距离截断下限
    bool   clamp_distance = true;
    // double inflation_radius = 0.0; // 整体外推，等效给障碍加半径
    double far_distance   = 10.0;  //带内无任何有效采样时的兜底值
    MinZConfig projection;
    Field2D    field_;
};
inline MinZResult reduceMinZ(const std::vector<double> &col, const MinZConfig &cfg) {
    MinZResult r;
    double best = std::numeric_limits<double>::max();

    for (const double d : col) {
        if (!std::isfinite(d)) {
            continue;                       // 无效采样直接跳过，不污染 min
        }
        ++r.n_sampled;
        if (d < 0.0) {
            ++r.n_blocked;
        }
        best = std::min(best, d);
    }

    if (r.n_sampled == 0) {
        r.distance = cfg.far_distance;
        r.valid    = false;
        return r;
    }

    r.valid   = true;
    r.blocked = (r.n_blocked > 0);

//     double v = best - cfg.inflation_radius;
//     if (!std::isfinite(v)) {
//         v = cfg.far_distance;
//     }
//     if (cfg.clamp_distance) {
//         v = std::clamp(v, std::min(0.0, cfg.min_distance), std::max(0.1, cfg.max_distance));
//     }
//     r.distance = v;
//     return r;
// }

    double v=best;

    class Field2D {
    public:
        void update(int width, int height, double resolution, const Eigen::Vector2d &origin,
                    std::vector<double> dist_m, std::vector<uint8_t> occupied);

        bool evaluate(const Eigen::Vector2d &pos, double &dist) const;

        int width()  const { return width_; }
        int height() const { return height_; }
        double resolution() const { return resolution_; }
        double maxDistance() const { return max_distance_; }
        const Eigen::Vector2d &origin() const { return origin_; }
        const std::vector<double>   &distances() const { return dist_m_; }
        const std::vector<uint8_t>  &occupied()  const { return occ_; }

    private:
        int width_ = 0, height_ = 0;
        double resolution_ = 0.0, max_distance_ = 0.0;
        Eigen::Vector2d origin_ = Eigen::Vector2d::Zero();
        std::vector<double>  dist_m_;
        std::vector<uint8_t> occ_;   ///< 1 = blocked（d_2D < 0），0 = 可通过
        mutable std::mutex mutex_;
};
    
};