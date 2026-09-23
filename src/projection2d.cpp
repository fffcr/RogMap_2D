#include <rog_map/projection2d.h>
#include <stdexcept>

namespace rog_map {

void Field2D::update(int width, int height, double resolution, const Eigen::Vector2d &origin,
                     std::vector<double> dist_m, std::vector<uint8_t> occupied,
                     std::vector<uint8_t> unknown) {
    if (width <= 0 || height <= 0 || resolution <= 0.0) {
        throw std::invalid_argument("Field2D::update: invalid grid metadata");
    }
    const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (dist_m.size() != expected || occupied.size() != expected || unknown.size() != expected) {
        throw std::invalid_argument("Field2D::update: size mismatch");
    }

    double mx = 0.0;
    for (const double d : dist_m) {
        mx = std::max(mx, d);
    }

std::lock_guard<std::mutex> lock(mutex_);
    width_ = width; height_ = height; resolution_ = resolution; origin_ = origin;
    max_distance_ = mx;
    dist_m_.swap(dist_m);
    occ_.swap(occupied);
    unk_.swap(unknown);
}

bool Field2D::evaluate(const Eigen::Vector2d &pos, double &dist) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (width_ <= 0 || height_ <= 0) {
        return false;
    }
    const int x = static_cast<int>(std::floor((pos.x() - origin_.x()) / resolution_));//世界坐标系到格子号
    const int y = static_cast<int>(std::floor((pos.y() - origin_.y()) / resolution_));
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return false;
    }
    dist = dist_m_[static_cast<size_t>(y) * static_cast<size_t>(width_) + static_cast<size_t>(x)];
    return true;
}

}  // namespace rog_map