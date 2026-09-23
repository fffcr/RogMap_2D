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
    bool   valid    = false; // 柱内是否取到过有效采样
    bool   blocked  = false; // 是否存在 d_3D < 0 的采样（该柱不可通过）
    int    n_sampled = 0;    // 有效采样数
    int    n_blocked = 0;    // 其中 d_3D < 0 的数量
};

/// 归约配置
struct MinZConfig {
    bool   enable = false;
    double max_distance   =  6.0;  // 正距离截断上限
    double min_distance   = -3.0;  // 负距离截断下限
    bool   clamp_distance = true;
    double far_distance   = 10.0;  // 带内无任何有效采样时的兜底值

    /// 输出二选一：true = 发布 2D 距离场(ESDF)，false = 发布 2D 栅格占用图。
    /// 只发布其中一种，由 rog_map_ros2 据此选 topic。
    bool output_esdf = false;
};

/// 纯函数：对一个柱子的 3D 距离序列取 min 
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

    double v = best;
    if (cfg.clamp_distance) {
        v = std::clamp(v, std::min(0.0, cfg.min_distance), std::max(0.1, cfg.max_distance));
    }
    r.distance = v;
    return r;
}

// 2D 距离场容器
class Field2D {
public:
    void update(int width, int height, double resolution, const Eigen::Vector2d &origin,
                std::vector<double> dist_m, std::vector<uint8_t> occupied);

    // 世界坐标查询。dist 正值自由、负值障碍内。越界返回 false。
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

}  // namespace rog_map
