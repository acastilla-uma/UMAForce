#pragma once

#include "fusionforce/route_destination.hpp"
#include "fusionforce/stability.hpp"
#include "fusionforce/truck_step.hpp"

#include <functional>
#include <iosfwd>

namespace fusionforce {

struct SimulationInput {
    CloudXyz origin{};
    rigid::State initial{};
    std::vector<Waypoint> route;
    double dt = .01;
    double duration = 2;
    double speed = 0;
    double steering = 0; // Radians.
    bool until_destination = false;
    bool lightweight = false;
};

struct SimulationFrame {
    double time;
    rigid::State state;
    CloudXyz global_position;
    RouteCommand command;
    double tangential_acceleration;
    double acceleration_error;
    TruckStepResult step;
    StabilityDiagnostics stability;
};

struct SimulationResult {
    std::vector<SimulationFrame> frames;
    std::vector<std::string> warnings;
    std::string status = "complete";
    std::optional<std::string> stopped_reason;
    std::size_t samples = 0;
    std::optional<bool> destination_reached;
    std::optional<double> distance_remaining;
    double time_limit = 0;
};

struct SimulationProgress {
    std::string_view stage;
    std::size_t processed;
    std::size_t total;
    double fraction;
    double time;
    double duration;
    std::size_t limit_steps;
    std::size_t estimated_steps;
    std::optional<double> distance_remaining;
    std::optional<std::size_t> waypoint_index;
    std::size_t waypoints_total;
    double dt = .01;
};

SimulationResult simulate_truck(const CloudStore& cloud, const TruckGeometry& truck,
                                const TruckTerrain& terrain, const SimulationInput& input,
                                const std::function<void(SimulationProgress)>& progress = {},
                                std::ostream* csv_output = nullptr);

} // namespace fusionforce
