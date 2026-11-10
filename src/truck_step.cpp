#include "fusionforce/truck_step.hpp"

#include <algorithm>
#include <cmath>

namespace fusionforce {
namespace {

CloudXyz global_position(CloudXyz origin, const rigid::Vec3& relative)
{
    for (std::size_t i = 0; i < origin.size(); ++i) {
        origin[i] += relative[i];
    }
    return origin;
}

template<std::size_t N>
bool finite(const std::array<float, N>& values)
{
    return std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); });
}

} // namespace

TruckContactMaterial TruckTerrain::material_at(const CloudPoint& point, const CloudLabels& labels) const
{
    // Zones are in global XYZ and the last matching zone takes precedence.
    for (auto zone = zones.rbegin(); zone != zones.rend(); ++zone) {
        const double dx = point.xyz[0] - zone->center[0];
        const double dy = point.xyz[1] - zone->center[1];
        const double dz = point.xyz[2] - zone->center[2];
        if (dx * dx + dy * dy + dz * dz <= zone->radius * zone->radius) {
            return {zone->terrain_class, classes.at(zone->terrain_class)};
        }
    }
    const auto label = labels.find(std::to_string(point.label));
    if (label != labels.end()) {
        const auto material = classes.find(label->second);
        if (material != classes.end()) {
            return {material->first, material->second};
        }
    }
    return {"desconocido", classes.at("desconocido")};
}

TruckStepResult TruckStepper::step(const CloudStore& cloud, const TruckGeometry& truck,
                                   const rigid::State& state, const rigid::EngineConfig& config,
                                   float speed, float steering, bool lightweight,
                                   const TruckTerrain& terrain)
{
    const auto vehicle_position = global_position(origin_, state.position);
    const auto world_geometry = transform_truck(truck, state, speed, steering, lightweight);
    TruckStepResult result{};
    result.global_cog = global_position(origin_, world_geometry.center_of_gravity);
    result.global_points.reserve(world_geometry.contacts.size());
    std::vector<const CloudPoint*> observations;
    observations.reserve(world_geometry.contacts.size());
    try {
        patch_.update(cloud, {vehicle_position[0], vehicle_position[1]});
        for (const auto& contact : world_geometry.contacts) {
            const auto position = global_position(origin_, contact.position);
            result.global_points.push_back(position);
            observations.push_back(&patch_.covered_point({position[0], position[1]}));
        }
    } catch (const std::invalid_argument& error) {
        throw TruckCoverageError(error.what());
    }

    // The float32 engine operates in the patch's XY frame and the original Z frame.
    // Transform again after recentering to match the original order of operations.
    rigid::State local_state = state;
    const float shift_x = static_cast<float>(patch_.center()[0] - origin_[0]);
    const float shift_y = static_cast<float>(patch_.center()[1] - origin_[1]);
    local_state.position[0] -= shift_x;
    local_state.position[1] -= shift_y;
    auto geometry = transform_truck(truck, local_state, speed, steering, lightweight);
    const auto& labels = cloud.metadata().label_mapping;
    result.contact_materials.reserve(geometry.contacts.size());
    for (std::size_t i = 0; i < geometry.contacts.size(); ++i) {
        auto& contact = geometry.contacts[i];
        result.contact_materials.push_back(terrain.material_at(*observations[i], labels));
        contact.terrain = patch_.sample(contact.position[0], contact.position[1],
                                       result.contact_materials.back().material);
    }
    result.physics = rigid::step(local_state, config, static_cast<float>(truck.mass),
                                 geometry.center_of_gravity, geometry.inertia, {1, 0, 0},
                                 geometry.contacts);
    result.moments.reserve(world_geometry.contacts.size());
    for (std::size_t i = 0; i < world_geometry.contacts.size(); ++i) {
        const auto& point = world_geometry.contacts[i].position;
        const auto& cog = world_geometry.center_of_gravity;
        const auto& contact = result.physics.contacts[i];
        const rigid::Vec3 lever{point[0] - cog[0], point[1] - cog[1], point[2] - cog[2]};
        const rigid::Vec3 force{contact.spring[0] + contact.friction[0],
                               contact.spring[1] + contact.friction[1],
                               contact.spring[2] + contact.friction[2]};
        result.moments.push_back({lever[1]*force[2] - lever[2]*force[1],
                                  lever[2]*force[0] - lever[0]*force[2],
                                  lever[0]*force[1] - lever[1]*force[0]});
    }
    auto& next = result.physics.state;
    next.position[0] += shift_x;
    next.position[1] += shift_y;
    if (!finite(next.position) || !finite(next.velocity) || !finite(next.orientation)
        || !finite(next.angular_velocity) || !finite(result.physics.acceleration)
        || !finite(result.physics.angular_acceleration)) {
        throw TruckInstabilityError("Simulación inestable; reduce el paso o cambia la pose");
    }
    return result;
}

} // namespace fusionforce
