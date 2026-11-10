#include "fusionforce/viewer_results.hpp"

#include <array>
#include <utility>

namespace fusionforce {
namespace {

using Json = nlohmann::json;
constexpr std::array<std::string_view, 4> corner_ids{
    "front_left", "front_right", "rear_left", "rear_right"
};

template <typename T>
Json nullable(const std::optional<T>& value)
{
    return value ? Json(*value) : Json(nullptr);
}

Json waypoint_json(const std::optional<Waypoint>& target)
{
    if (!target) {
        return nullptr;
    }
    return {{"x", target->x}, {"y", target->y}, {"z", target->z},
            {"speed", target->speed}, {"acceleration", target->acceleration}};
}

Json stability_json(const StabilityDiagnostics& diagnostics)
{
    Json result = Json::object();
    for (const auto& [name, entry] : diagnostics) {
        Json params = Json::object();
        for (const auto& [key, value] : entry.params) {
            params[key] = nullable(value);
        }
        const auto value = entry.value
            ? std::visit([](const auto& item) { return Json(item); }, *entry.value)
            : Json(nullptr);
        result[name] = {{"value", value}, {"params", std::move(params)}, {"reason", nullable(entry.reason)}};
    }
    return result;
}

} // namespace

Json viewer_frame_json(const SimulationFrame& frame, bool lightweight)
{
    const auto& physics = frame.step.physics;
    Json normal = Json::array();
    Json friction = Json::array();
    Json contact = Json::array();
    for (const auto& point : physics.contacts) {
        normal.push_back(point.spring);
        friction.push_back(point.friction);
        contact.push_back(point.contact_weight);
    }
    Json terrain = Json::array();
    for (const auto& material : frame.step.contact_materials) {
        terrain.push_back(material.terrain_class);
    }

    Json result{
        {"time", frame.time}, {"points", frame.step.global_points},
        {"normal", std::move(normal)}, {"friction", std::move(friction)},
        {"moments", frame.step.moments}, {"contact", std::move(contact)},
        {"acceleration", physics.acceleration}, {"angular_acceleration", physics.angular_acceleration},
        {"torque", physics.torque}, {"position", frame.global_position},
        {"orientation", frame.state.orientation}, {"velocity", frame.state.velocity},
        {"angular_velocity", frame.state.angular_velocity}, {"terrain", std::move(terrain)},
        {"target", waypoint_json(frame.command.target)}, {"tracking_error", frame.command.tracking_error},
        {"speed_error", frame.command.speed_error}, {"tangential_acceleration", frame.tangential_acceleration},
        {"acceleration_error", frame.acceleration_error}, {"stability", stability_json(frame.stability)},
    };
    if (lightweight) {
        result["contact_ids"] = corner_ids;
    }
    return result;
}

Json viewer_result_json(const SimulationResult& result, std::string_view vehicle,
                        bool lightweight, bool record_csv)
{
    Json frames = Json::array();
    for (const auto& frame : result.frames) {
        frames.push_back(viewer_frame_json(frame, lightweight));
    }
    Json response{
        {"frames", std::move(frames)}, {"warnings", result.warnings}, {"status", result.status},
        {"samples", result.samples}, {"csv", record_csv ? Json("/api/export.csv") : Json(nullptr)},
        {"frame", "global XYZ (m), Z vertical"}, {"vehicle", vehicle},
        {"contact_mode", lightweight ? "four_corners" : "full"},
    };
    if (result.destination_reached) {
        response["destination_reached"] = *result.destination_reached;
        response["stopped_reason"] = nullable(result.stopped_reason);
        response["distance_remaining"] = nullable(result.distance_remaining);
        response["time_limit"] = result.time_limit;
    }
    return response;
}

Json viewer_progress_json(const SimulationProgress& progress, double dt)
{
    Json result{
        {"stage", progress.stage}, {"processed", progress.processed}, {"total", progress.total},
        {"fraction", progress.fraction}, {"time", progress.time}, {"duration", progress.duration},
        {"until_destination", progress.waypoint_index.has_value()}, {"dt", dt},
        {"estimated_steps", progress.estimated_steps}, {"limit_steps", progress.limit_steps},
    };
    if (progress.waypoint_index) {
        result["time_limit"] = progress.duration;
        result["distance_remaining"] = nullable(progress.distance_remaining);
        result["waypoint_index"] = *progress.waypoint_index;
        result["waypoints_total"] = progress.waypoints_total;
    }
    return result;
}

} // namespace fusionforce
