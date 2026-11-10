#include "fusionforce/terrain_patch.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace fusionforce {
namespace {

constexpr double resolution = 0.2;
constexpr double half_extent = 16;
constexpr std::size_t side = 161;

double lattice_coordinate(double value)
{
    const double scaled = value / resolution;
    const double lower = std::floor(scaled);
    if (scaled - lower == 0.5) {
        return (std::fmod(lower, 2.0) == 0 ? lower : lower + 1) * resolution;
    }
    return std::round(scaled) * resolution;
}

} // namespace

void TerrainPatch::update(const CloudStore& cloud, CloudXy position)
{
    if (grid_ && std::hypot(position[0] - center_[0], position[1] - center_[1]) < 4) {
        return;
    }

    TerrainPatch candidate(origin_);
    candidate.center_ = {lattice_coordinate(position[0]), lattice_coordinate(position[1])};
    std::vector<CloudXy> queries;
    queries.reserve(side * side);
    for (std::size_t y = 0; y < side; ++y) {
        for (std::size_t x = 0; x < side; ++x) {
            queries.push_back({x * resolution - half_extent + candidate.center_[0],
                               y * resolution - half_extent + candidate.center_[1]});
        }
    }
    const auto observations = cloud.upper_xy(queries, 0.35);
    for (const auto& observation : observations) {
        if (std::isfinite(observation.distance)) {
            candidate.points_.push_back(observation.point);
        }
    }
    if (candidate.points_.size() < 3) {
        throw std::invalid_argument("Huella fuera de cobertura: no hay datos locales suficientes");
    }

    candidate.tree_.resize(candidate.points_.size());
    std::iota(candidate.tree_.begin(), candidate.tree_.end(), 0);
    candidate.partition(0, candidate.tree_.size(), 0);
    std::vector<float> heights;
    heights.reserve(queries.size());
    for (std::size_t i = 0; i < queries.size(); ++i) {
        const auto& observation = observations[i];
        double distance;
        const auto& point = std::isfinite(observation.distance)
            ? observation.point : candidate.points_[candidate.nearest(queries[i], distance)];
        heights.push_back(static_cast<float>(point.xyz[2] - origin_[2]));
    }
    candidate.grid_.emplace(side, side, resolution, half_extent, std::move(heights));
    *this = std::move(candidate);
}

void TerrainPatch::partition(std::size_t first, std::size_t last, std::size_t axis)
{
    if (last - first < 2) {
        return;
    }
    const auto middle = first + (last - first) / 2;
    std::nth_element(tree_.begin() + first, tree_.begin() + middle, tree_.begin() + last,
                     [&](std::size_t a, std::size_t b) {
        const double left = points_[a].xyz[axis];
        const double right = points_[b].xyz[axis];
        return left == right ? a < b : left < right;
    });
    partition(first, middle, 1 - axis);
    partition(middle + 1, last, 1 - axis);
}

std::size_t TerrainPatch::nearest(CloudXy point, double& squared_distance) const
{
    std::size_t index = 0;
    squared_distance = std::numeric_limits<double>::infinity();
    search(point, 0, tree_.size(), 0, index, squared_distance);
    return index;
}

void TerrainPatch::search(CloudXy point, std::size_t first, std::size_t last, std::size_t axis,
                          std::size_t& index, double& squared_distance) const
{
    if (first == last) {
        return;
    }
    const auto middle = first + (last - first) / 2;
    const auto current = tree_[middle];
    const auto& xyz = points_[current].xyz;
    const double dx = xyz[0] - point[0];
    const double dy = xyz[1] - point[1];
    const double distance = dx * dx + dy * dy;
    if (distance < squared_distance || (distance == squared_distance && current < index)) {
        index = current;
        squared_distance = distance;
    }
    const double offset = xyz[axis] - point[axis];
    if (offset > 0) {
        search(point, first, middle, 1 - axis, index, squared_distance);
        if (offset * offset <= squared_distance) {
            search(point, middle + 1, last, 1 - axis, index, squared_distance);
        }
    } else {
        search(point, middle + 1, last, 1 - axis, index, squared_distance);
        if (offset * offset <= squared_distance) {
            search(point, first, middle, 1 - axis, index, squared_distance);
        }
    }
}

const CloudPoint& TerrainPatch::covered_point(CloudXy position) const
{
    double distance;
    const auto index = nearest(position, distance);
    if (distance > 0.8 * 0.8) {
        throw std::invalid_argument("Huella fuera de cobertura: se requieren datos a menos de 0,8 m");
    }
    return points_[index];
}

rigid::TerrainSample TerrainPatch::sample(float x, float y, rigid::Material material) const
{
    return grid_.value().sample(x, y, material);
}

} // namespace fusionforce
