#include "fusionforce/simulation.hpp"
#include "fusionforce/simulation_csv.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace fusionforce {
namespace {

template <std::size_t N>
std::array<double, N> promote(const std::array<float, N>& values)
{
    std::array<double, N> result;
    std::copy(values.begin(), values.end(), result.begin());
    return result;
}

} // namespace

SimulationResult simulate_truck(const CloudStore& cloud, const TruckGeometry& truck,
                                const TruckTerrain& terrain, const SimulationInput& input,
                                const std::function<void(SimulationProgress)>& progress,
                                std::ostream* csv_output)
{
    if (!(input.dt >= .0005 && input.dt <= .01) ||
        (!input.until_destination && !(input.duration >= 0 && input.duration <= 60))) {
        throw std::invalid_argument("Paso o duración de simulación inválidos");
    }
    const CloudXy origin_xy{input.origin[0], input.origin[1]};
    RouteController controller(input.route, origin_xy, input.dt, input.until_destination,
                               input.speed, input.steering);
    const auto& destination = controller.destination();
    SimulationResult result;
    result.time_limit = destination ? destination->time_limit() : input.duration;
    const auto steps = static_cast<std::size_t>(result.time_limit / input.dt);
    const auto estimated_steps = destination
        ? static_cast<std::size_t>(std::ceil(std::min(result.time_limit, destination->estimated_duration()) / input.dt)) + 1
        : steps + 1;
    const auto stride = std::max<std::size_t>(1, static_cast<std::size_t>(.04 / input.dt));
    const auto report = [&](std::string_view stage, std::size_t total, double fraction, double time) {
        if (progress) {
            progress({stage, result.samples, total, fraction, time, result.time_limit, steps + 1,
                      estimated_steps, destination ? std::optional<double>(result.distance_remaining
                          .value_or(destination->remaining())) : std::nullopt,
                      destination ? std::optional<std::size_t>(std::min(destination->index(), input.route.size() - 1))
                                  : std::nullopt,
                      input.route.size(), input.dt});
        }
    };
    report("preparation", estimated_steps, 0, 0);

    rigid::EngineConfig config;
    config.dt = static_cast<float>(input.dt);
    config.torque_limit = 1e7f;
    Matrix3 inertia{};
    for (std::size_t i = 0; i < inertia.size(); ++i) {
        inertia[i][i] = truck.moments_of_inertia[i];
    }
    std::vector<Vector3> wheel_centers;
    for (const auto& center : truck.wheel_centers) {
        wheel_centers.push_back(promote(center));
    }
    StabilityMonitor stability(truck.mass, inertia, promote(truck.center_of_gravity), wheel_centers,
                               truck.radius, input.lightweight ? 4 : truck.wheel_centers.size() * truck.ring.size());
    TruckStepper stepper(input.origin);
    std::optional<SimulationCsvWriter> csv;
    if (csv_output) {
        csv.emplace(*csv_output, input.lightweight);
    }
    auto state = input.initial;
    std::optional<SimulationFrame> last_frame;
    for (std::size_t i = 0; i <= steps; ++i) {
        const double time = i * input.dt;
        CloudXyz position = input.origin;
        for (std::size_t axis = 0; axis < position.size(); ++axis) {
            position[axis] += state.position[axis];
        }
        const auto rotation = rigid::rotation_matrix(state.orientation);
        const auto command = controller.update(position, {state.velocity[0], state.velocity[1]},
                                               std::atan2(rotation[1][0], rotation[0][0]), time);
        if (destination && destination->reached()) {
            result.status = "destination_reached";
        } else if (destination && time - destination->last_improvement() >= RouteDestination::stall_seconds) {
            result.status = "destination_not_reached";
            result.stopped_reason = "stall";
        }

        TruckStepResult step;
        try {
            step = stepper.step(cloud, truck, state, config, static_cast<float>(command.speed),
                                static_cast<float>(command.steering), input.lightweight, terrain);
        } catch (const TruckCoverageError& error) {
            if (i == 0) {
                throw;
            }
            result.warnings.push_back(error.what());
            result.status = destination ? "destination_not_reached" : "coverage_stop";
            if (destination) {
                result.stopped_reason = "coverage";
                result.distance_remaining = std::hypot(input.route.back().x - position[0],
                                                        input.route.back().y - position[1]);
            }
            break;
        }
        const auto next = step.physics.state;
        std::vector<Vector3> forces;
        forces.reserve(step.physics.contacts.size());
        for (const auto& contact : step.physics.contacts) {
            forces.push_back({contact.spring[0] + contact.friction[0],
                              contact.spring[1] + contact.friction[1],
                              contact.spring[2] + contact.friction[2]});
        }
        auto diagnostics = stability.update(time, step.global_points, forces, step.global_cog,
                                            promote(state.orientation), promote(state.velocity),
                                            promote(step.physics.acceleration), promote(state.angular_velocity));
        const float speed_xy = std::hypot(state.velocity[0], state.velocity[1]);
        const auto tx = speed_xy > .1 ? state.velocity[0] / speed_xy : rotation[0][0];
        const auto ty = speed_xy > .1 ? state.velocity[1] / speed_xy : rotation[1][0];
        const auto norm = std::max(std::hypot(tx, ty), 1e-9f);
        const double tangential = (step.physics.acceleration[0] * tx + step.physics.acceleration[1] * ty) / norm;
        const double acceleration_error = command.target ? command.target->acceleration - tangential : 0;
        last_frame = SimulationFrame{time, state, position, command, tangential, acceleration_error,
                                     std::move(step), std::move(diagnostics)};
        if (csv) {
            csv->write(*last_frame);
        }
        if (i % stride == 0 || i == steps) {
            result.frames.push_back(std::move(*last_frame));
            last_frame.reset();
        }
        ++result.samples;
        state = next;
        const auto interval = std::max<std::size_t>(1, std::min(estimated_steps / 200,
                                                            static_cast<std::size_t>(.1 / input.dt)));
        if (result.samples == 1 || result.samples == estimated_steps || result.samples % interval == 0) {
            report("simulation", estimated_steps, std::min(.98, .98 * result.samples / estimated_steps), time);
        }
        if (result.status != "complete") {
            break;
        }
    }
    if (last_frame) {
        result.frames.push_back(std::move(*last_frame));
    }
    if (destination) {
        if (result.status == "complete") {
            result.status = "destination_not_reached";
            result.stopped_reason = "timeout";
        }
        result.destination_reached = result.status == "destination_reached";
        if (!result.distance_remaining) {
            result.distance_remaining = destination->remaining();
        }
    }
    const double last_time = result.frames.empty() ? 0 : result.frames.back().time;
    report("finalizing", destination ? result.samples : estimated_steps, .99, last_time);
    if (!input.route.empty() && !result.frames.empty()) {
        const auto& last = result.frames.back();
        if (std::any_of(result.frames.begin(), result.frames.end(), [](const SimulationFrame& frame) {
                return frame.command.tracking_error > 2;
            })) {
            result.warnings.emplace_back("Desvío de ruta superior a 2 m");
        }
        if (result.time_limit >= 1 && std::abs(last.command.speed_error) > 1) {
            result.warnings.emplace_back("Velocidad objetivo no alcanzada (error > 1 m/s)");
        }
        if (std::any_of(result.frames.begin(), result.frames.end(), [](const SimulationFrame& frame) {
                return std::abs(frame.acceleration_error) > 1;
            })) {
            result.warnings.emplace_back("Aceleración objetivo no alcanzada (error > 1 m/s²)");
        }
        const auto& end = input.route.back();
        if (destination ? !*result.destination_reached
                        : std::hypot(last.global_position[0] - end.x, last.global_position[1] - end.y,
                                     last.global_position[2] - end.z) > .5) {
            result.warnings.emplace_back("Punto final no alcanzado: tolerancia 0,5 m");
        }
    }
    if (destination && !*result.destination_reached) {
        if (result.stopped_reason == "stall") {
            result.warnings.emplace_back("Destino no alcanzado: 15 s sin avance hacia el siguiente punto");
        } else if (result.stopped_reason == "coverage") {
            result.warnings.emplace_back("Destino no alcanzado: la ruta sale de la cobertura");
        } else {
            result.warnings.emplace_back("Destino no alcanzado: límite de tiempo de la ruta agotado");
        }
    }
    report("complete", destination ? result.samples : estimated_steps, 1, last_time);
    return result;
}

} // namespace fusionforce
