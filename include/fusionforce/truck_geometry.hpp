#pragma once

#include "fusionforce/rigid_body.hpp"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace fusionforce {

using TruckPoint = std::array<float, 3>;

struct TruckGeometry {
    std::string vehicle_id;
    double mass;
    TruckPoint center_of_gravity;
    TruckPoint moments_of_inertia;
    double radius;
    double width;
    std::vector<TruckPoint> wheel_centers;
    std::vector<float> steering_factors;
    std::vector<TruckPoint> ring;
    std::vector<TruckPoint> body;
    std::array<TruckPoint, 4> corners;
    std::array<float, 4> corner_steering_factors;
};

TruckGeometry load_truck_geometry(const std::filesystem::path& man_assets, const std::string& vehicle_id);

struct TruckContactGeometry {
    rigid::Vec3 center_of_gravity;
    rigid::Mat3 inertia;
    std::vector<rigid::ContactPoint> contacts;
};

TruckContactGeometry transform_truck(const TruckGeometry& truck, const rigid::State& state,
                                     float speed, float steering, bool lightweight);

} // namespace fusionforce
