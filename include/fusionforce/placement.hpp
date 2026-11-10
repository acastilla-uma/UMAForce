#pragma once

#include <array>
#include <functional>
#include <string>
#include <vector>

namespace fusionforce {

inline constexpr double support_radius = 0.35;
inline constexpr double coverage_radius = 0.8;
inline constexpr double drop_clearance = 0.2;

using PlacementPoint = std::array<double, 3>;
using PlacementXY = std::array<double, 2>;
using PlacementBounds = std::array<double, 4>;
using PlacementRegion = std::function<std::vector<PlacementPoint>(const PlacementBounds&)>;

struct PlacementGeometry {
    std::vector<PlacementPoint> centers;
    std::vector<PlacementPoint> ring;
    std::vector<PlacementPoint> body;
    double radius = 0;
    double width = 0;
    std::string vehicle_id = "MAN_10t";
};

struct PlacementSeed {
    PlacementPoint position;
    std::array<double, 4> orientation; // w, x, y, z
    double roll = 0;
    double pitch = 0;
    double yaw;
    std::vector<PlacementPoint> wheel_contacts;
    std::vector<PlacementPoint> wheel_centers;
    double tire_radius;
    double tire_width;
    double support_clearance = drop_clearance;
    std::string vehicle;
};

// Returns the level drop seed used before the physical settling pass.
PlacementSeed place_vehicle(const PlacementRegion& query, const PlacementGeometry& engine,
                            PlacementXY xy, double yaw);

} // namespace fusionforce
