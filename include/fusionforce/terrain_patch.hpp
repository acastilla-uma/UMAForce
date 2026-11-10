#pragma once

#include "fusionforce/cloud_store.hpp"
#include "fusionforce/rigid_body.hpp"

#include <optional>

namespace fusionforce {

class TerrainPatch {
public:
    explicit TerrainPatch(CloudXyz origin) : origin_(origin) {}

    void update(const CloudStore& cloud, CloudXy position);
    const CloudXy& center() const noexcept { return center_; }
    const CloudPoint& covered_point(CloudXy position) const;
    rigid::TerrainSample sample(float x, float y, rigid::Material material) const;

private:
    std::size_t nearest(CloudXy point, double& squared_distance) const;
    void partition(std::size_t first, std::size_t last, std::size_t axis);
    void search(CloudXy point, std::size_t first, std::size_t last, std::size_t axis,
                std::size_t& index, double& squared_distance) const;

    CloudXyz origin_;
    CloudXy center_{};
    std::vector<CloudPoint> points_;
    std::vector<std::size_t> tree_;
    std::optional<rigid::HeightGrid> grid_;
};

} // namespace fusionforce
