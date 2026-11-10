#include "fusionforce/placement.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fusionforce {
namespace {

bool finite(const PlacementPoint& point)
{
    return std::all_of(point.begin(), point.end(), [](double value) { return std::isfinite(value); });
}

} // namespace

PlacementSeed place_vehicle(const PlacementRegion& query, const PlacementGeometry& engine,
                            PlacementXY xy, double yaw)
{
    if (!std::isfinite(xy[0]) || !std::isfinite(xy[1])) {
        throw std::invalid_argument("Selecciona una posición XY válida sobre el terreno");
    }
    if (!std::isfinite(yaw)) {
        throw std::invalid_argument("La orientación debe ser un ángulo válido");
    }
    yaw = std::fmod(yaw + 180.0, 360.0);
    if (yaw < 0) {
        yaw += 360.0;
    }
    yaw -= 180.0;

    if (engine.centers.size() < 4 || engine.ring.empty() ||
        !std::all_of(engine.centers.begin(), engine.centers.end(), finite) ||
        !std::all_of(engine.ring.begin(), engine.ring.end(), finite) ||
        !std::isfinite(engine.radius) || !std::isfinite(engine.width) ||
        std::min(engine.radius, engine.width) <= 0) {
        throw std::invalid_argument("Geometría de vehículo no compatible");
    }

    const double infinity = std::numeric_limits<double>::infinity();
    PlacementPoint low{infinity, infinity, infinity};
    PlacementXY high{-infinity, -infinity};
    double reach = 0;
    const auto include = [&](const PlacementPoint& point) {
        for (std::size_t axis = 0; axis < low.size(); ++axis) {
            low[axis] = std::min(low[axis], point[axis]);
        }
        for (std::size_t axis = 0; axis < high.size(); ++axis) {
            high[axis] = std::max(high[axis], point[axis]);
        }
        reach = std::max(reach, std::hypot(point[0], point[1]));
    };
    for (const auto& center : engine.centers) {
        for (const auto& sample : engine.ring) {
            include({center[0] + sample[0], center[1] + sample[1], center[2] + sample[2]});
        }
    }
    for (const auto& point : engine.body) {
        include(point);
    }
    reach += coverage_radius;
    const auto points = query({xy[0] - reach, xy[1] - reach, xy[0] + reach, xy[1] + reach});

    constexpr double degrees_to_half_radians = 3.14159265358979323846 / 360.0;
    const double qw = std::cos(yaw * degrees_to_half_radians);
    const double qz = std::sin(yaw * degrees_to_half_radians);
    const double cosine = qw * qw - qz * qz;
    const double sine = 2 * qw * qz;
    bool observed = false;
    double region_top = -infinity;
    double footprint_top = -infinity;
    for (const auto& point : points) {
        const double dx = point[0] - xy[0];
        const double dy = point[1] - xy[1];
        observed = observed || std::hypot(dx, dy) <= coverage_radius;
        region_top = std::max(region_top, point[2]);
        const double local_x = cosine * dx + sine * dy;
        const double local_y = -sine * dx + cosine * dy;
        if (local_x >= low[0] - support_radius && local_x <= high[0] + support_radius &&
            local_y >= low[1] - support_radius && local_y <= high[1] + support_radius) {
            footprint_top = std::max(footprint_top, point[2]);
        }
    }
    if (!observed) {
        throw std::invalid_argument("Posición no válida: no hay puntos en el lugar seleccionado");
    }
    const double terrain_top = footprint_top == -infinity ? region_top : footprint_top;
    const double z = terrain_top - low[2] + drop_clearance;
    PlacementSeed seed{};
    seed.position = {xy[0], xy[1], z};
    seed.orientation = {qw, 0, 0, qz};
    seed.yaw = yaw;
    seed.wheel_centers = engine.centers;
    seed.tire_radius = engine.radius;
    seed.tire_width = engine.width;
    seed.vehicle = engine.vehicle_id;
    seed.wheel_contacts.reserve(engine.centers.size());
    for (const auto& center : engine.centers) {
        seed.wheel_contacts.push_back({
            cosine * center[0] - sine * center[1] + xy[0],
            sine * center[0] + cosine * center[1] + xy[1],
            center[2] - engine.radius + z,
        });
    }
    return seed;
}

} // namespace fusionforce
