#include "fusionforce/route_destination.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace fusionforce {
namespace {

double distance(const Waypoint& waypoint, RoutePosition position)
{
    return std::hypot(waypoint.x - position[0], waypoint.y - position[1]);
}

} // namespace

RouteDestination::RouteDestination(std::vector<Waypoint> route, RoutePosition origin)
    : route_(std::move(route)), remaining_after_waypoint_(route_.size()),
      best_distance_(std::numeric_limits<double>::infinity())
{
    if (route_.empty()) {
        throw std::invalid_argument("La trayectoria debe tener al menos un punto");
    }
    for (const auto& waypoint : route_) {
        const double length = distance(waypoint, origin);
        remaining_ += length;
        estimated_duration_ += length / std::clamp(waypoint.speed, 0.25, 20.0);
        origin = {waypoint.x, waypoint.y};
    }
    for (std::size_t i = route_.size() - 1; i > 0; --i) {
        remaining_after_waypoint_[i - 1] =
            remaining_after_waypoint_[i] + distance(route_[i], {route_[i - 1].x, route_[i - 1].y});
    }
    time_limit_ = std::clamp(estimated_duration_ * 3 + 20, 30.0, 1800.0);
}

const Waypoint* RouteDestination::observe(RoutePosition position, double time)
{
    while (!reached() && distance(route_[index_], position) <= tolerance) {
        ++index_;
        best_distance_ = std::numeric_limits<double>::infinity();
        last_improvement_ = time;
    }
    if (reached()) {
        remaining_ = 0;
        return nullptr;
    }
    const double current_distance = distance(route_[index_], position);
    remaining_ = current_distance + remaining_after_waypoint_[index_];
    if (current_distance < best_distance_ - 0.05) {
        best_distance_ = current_distance;
        last_improvement_ = time;
    }
    return &route_[index_];
}

RouteController::RouteController(std::vector<Waypoint> route, RoutePosition origin, double dt,
                                 bool until_destination, double speed, double steering)
    : route_(std::move(route)), origin_(origin), dt_(dt), speed_(speed), steering_(steering)
{
    if (until_destination) {
        if (route_.size() < 2) {
            throw std::invalid_argument("Se requieren al menos dos puntos para llegar al destino");
        }
        destination_.emplace(route_, origin);
    }
    times_.push_back(0);
    for (std::size_t i = 1; i < route_.size(); ++i) {
        const auto& a = route_[i - 1];
        const auto& b = route_[i];
        const double segment = distance(b, {a.x, a.y});
        times_.push_back(times_.back() + std::max(dt_, segment / std::max(0.1, (a.speed + b.speed) / 2)));
    }
}

std::optional<Waypoint> RouteController::target_at(double time) const
{
    if (route_.empty()) {
        return std::nullopt;
    }
    const auto end = std::upper_bound(times_.begin() + 1, times_.end(), time);
    const auto i = static_cast<std::size_t>(end - times_.begin() - 1);
    const auto& a = route_[i];
    if (i == route_.size() - 1) {
        return a;
    }
    const auto& b = route_[i + 1];
    const double alpha = std::clamp((time - times_[i]) / (times_[i + 1] - times_[i]), 0.0, 1.0);
    const auto blend = [alpha](double x, double y) { return x * (1 - alpha) + y * alpha; };
    return Waypoint{blend(a.x, b.x), blend(a.y, b.y), blend(a.z, b.z),
                    blend(a.speed, b.speed), blend(a.acceleration, b.acceleration)};
}

RouteCommand RouteController::update(const std::array<double, 3>& position, RoutePosition velocity,
                                     double heading, double time)
{
    RouteCommand command{std::nullopt, speed_, steering_};
    const RoutePosition xy{position[0], position[1]};
    if (destination_) {
        if (const auto* target = destination_->observe(xy, time)) {
            command.target = *target;
        }
    } else {
        command.target = target_at(time);
    }
    if (!command.target) {
        return command;
    }
    const auto& target = *command.target;
    const double distance_xy = distance(target, xy);
    double desired = std::min(20.0, target.speed + std::min(3.0, distance_xy));
    const auto look = destination_ ? target : *target_at(time + 0.7);
    const double error = std::atan2(look.y - xy[1], look.x - xy[0]) - heading;
    constexpr double steering_limit = 39 * std::numbers::pi / 180;
    command.steering = std::clamp(std::atan2(std::sin(error), std::cos(error)), -steering_limit, steering_limit);
    command.tracking_error = std::hypot(distance_xy, target.z - position[2]);
    if (destination_) {
        const auto index = destination_->index();
        const auto& previous = route_[index == 0 ? 0 : index - 1];
        const RoutePosition start = index == 0 ? origin_ : RoutePosition{previous.x, previous.y};
        const double dx = target.x - start[0];
        const double dy = target.y - start[1];
        const double projection = ((xy[0] - start[0]) * dx + (xy[1] - start[1]) * dy)
            / std::max(dx * dx + dy * dy, 1e-12);
        const double fraction = std::clamp(projection, 0.0, 1.0);
        command.tracking_error = std::hypot(xy[0] - (start[0] + fraction * dx),
                                            xy[1] - (start[1] + fraction * dy));
        const double cruise = std::max(0.25, (previous.speed + target.speed) / 2);
        desired = std::min({20.0, cruise, std::sqrt(2 * std::max(0.5, std::abs(target.acceleration)) * distance_xy)});
    } else if (time >= times_.back() && command.tracking_error < 0.5) {
        desired = 0;
    }
    const double acceleration = std::clamp(target.acceleration + 2 * (desired - speed_), -20.0, 20.0);
    speed_ = std::clamp(speed_ + acceleration * dt_, -20.0, 20.0);
    command.speed = speed_;
    command.speed_error = target.speed - std::hypot(velocity[0], velocity[1]);
    return command;
}

} // namespace fusionforce
