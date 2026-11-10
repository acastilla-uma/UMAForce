#include "fusionforce/viewer_results.hpp"

#include <iostream>
#include <stdexcept>

namespace {

using nlohmann::json;
using namespace fusionforce;

void equal(const json& actual, const json& expected)
{
    if (actual != expected) {
        throw std::runtime_error("Expected " + expected.dump() + ", got " + actual.dump());
    }
}

void frame_contract()
{
    SimulationFrame frame{};
    frame.time = .125;
    frame.global_position = {100.5, 200, 300};
    frame.state.position = {.5, 0, 0};
    frame.state.orientation = {.5f, .5f, .5f, .5f};
    frame.state.velocity = {1, 2, 3};
    frame.state.angular_velocity = {4, 5, 6};
    frame.step.physics.acceleration = {7, 8, 9};
    frame.step.physics.angular_acceleration = {10, 11, 12};
    frame.step.physics.torque = {13, 14, 15};
    frame.step.global_points.push_back({100, 200, 300});
    frame.step.physics.contacts.push_back({{0, 0, 20}, {2, 1, 0}, .25f});
    frame.step.moments.push_back({0, 10, -.5f});
    frame.step.contact_materials.push_back({"grava", {}});
    frame.command.target = Waypoint{100, 201, 300, 2.5, .5};
    frame.command.tracking_error = .5;
    frame.command.speed_error = -.125;
    frame.tangential_acceleration = .25;
    frame.acceleration_error = .25;
    frame.stability["ltr"].value = .125;
    frame.stability["pltr"].reason = "Se necesita una muestra anterior para la derivada causal";
    frame.stability["pltr"].params["ay_dot_m_s3"] = std::nullopt;
    frame.stability["zml"].value = std::array<double, 2>{.25, .5};
    equal(viewer_frame_json(frame, false), json::parse(R"({
        "time": 0.125, "points": [[100, 200, 300]], "normal": [[0, 0, 20]],
        "friction": [[2, 1, 0]], "moments": [[0, 10, -0.5]], "contact": [0.25],
        "acceleration": [7, 8, 9], "angular_acceleration": [10, 11, 12],
        "torque": [13, 14, 15], "position": [100.5, 200, 300],
        "orientation": [0.5, 0.5, 0.5, 0.5], "velocity": [1, 2, 3],
        "angular_velocity": [4, 5, 6], "terrain": ["grava"],
        "target": {"x": 100, "y": 201, "z": 300, "speed": 2.5, "acceleration": 0.5},
        "tracking_error": 0.5, "speed_error": -0.125, "tangential_acceleration": 0.25,
        "acceleration_error": 0.25,
        "stability": {
            "ltr": {"value": 0.125, "reason": null, "params": {}},
            "pltr": {"value": null, "reason": "Se necesita una muestra anterior para la derivada causal",
                     "params": {"ay_dot_m_s3": null}},
            "zml": {"value": [0.25, 0.5], "reason": null, "params": {}}
        }
    })"));
    frame.command.target.reset();
    const auto lightweight = viewer_frame_json(frame, true);
    equal(lightweight.at("target"), nullptr);
    equal(lightweight.at("contact_ids"), {"front_left", "front_right", "rear_left", "rear_right"});
}

void result_contract()
{
    SimulationResult result;
    equal(viewer_result_json(result, "MAN_10t", false, false), json::parse(R"({
        "frames": [], "warnings": [], "status": "complete", "samples": 0, "csv": null,
        "frame": "global XYZ (m), Z vertical", "vehicle": "MAN_10t", "contact_mode": "full"
    })"));
    result.status = "destination_not_reached";
    result.destination_reached = false;
    result.stopped_reason = "stall";
    result.distance_remaining = 3;
    result.time_limit = 60;
    result.samples = 100;
    result.warnings = {"warning"};
    const auto routed = viewer_result_json(result, "MAN_5t", true, true);
    equal(routed, json::parse(R"({
        "frames": [], "warnings": ["warning"], "status": "destination_not_reached",
        "samples": 100, "csv": "/api/export.csv", "frame": "global XYZ (m), Z vertical",
        "vehicle": "MAN_5t", "contact_mode": "four_corners", "destination_reached": false,
        "stopped_reason": "stall", "distance_remaining": 3, "time_limit": 60
    })"));
    result.destination_reached = true;
    result.stopped_reason.reset();
    result.frames.emplace_back();
    const auto reached = viewer_result_json(result, "MAN_5t", true, true);
    equal(reached.at("stopped_reason"), nullptr);
    equal(reached.at("frames").size(), 1);
    equal(reached.at("frames").at(0), viewer_frame_json(result.frames.front(), true));
}

void progress_contract()
{
    const SimulationProgress manual{"preparation", 0, 201, 0, 0, 2, 201, 201, {}, {}, 0};
    equal(viewer_progress_json(manual, .01), json::parse(R"({
        "stage": "preparation", "processed": 0, "total": 201, "fraction": 0,
        "time": 0, "duration": 2, "until_destination": false, "dt": 0.01,
        "estimated_steps": 201, "limit_steps": 201
    })"));
    const SimulationProgress route{"finalizing", 101, 101, .99, 1, 60, 6001, 201, 2.5, 0, 3};
    equal(viewer_progress_json(route, .01), json::parse(R"({
        "stage": "finalizing", "processed": 101, "total": 101, "fraction": 0.99,
        "time": 1, "duration": 60, "until_destination": true, "dt": 0.01,
        "estimated_steps": 201, "limit_steps": 6001, "time_limit": 60,
        "distance_remaining": 2.5, "waypoint_index": 0, "waypoints_total": 3
    })"));
}

} // namespace

int main()
{
    try {
        frame_contract();
        result_contract();
        progress_contract();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
