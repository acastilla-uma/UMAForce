#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace fusionforce {

using RoutePosition = std::array<double, 2>;

struct Waypoint {
    double x;
    double y;
    double z;
    double speed;
    double acceleration;
};

class RouteDestination {
public:
    static constexpr double tolerance = 0.5;
    static constexpr double stall_seconds = 15.0;

    RouteDestination(std::vector<Waypoint> route, RoutePosition origin);
    const Waypoint* observe(RoutePosition position, double time);

    bool reached() const noexcept { return index_ == route_.size(); }
    double remaining() const noexcept { return remaining_; }
    double last_improvement() const noexcept { return last_improvement_; }
    double estimated_duration() const noexcept { return estimated_duration_; }
    double time_limit() const noexcept { return time_limit_; }
    std::size_t index() const noexcept { return index_; }

private:
    std::vector<Waypoint> route_;
    std::vector<double> remaining_after_waypoint_;
    std::size_t index_ = 0;
    double last_improvement_ = 0;
    double best_distance_;
    double estimated_duration_ = 0;
    double time_limit_;
    double remaining_ = 0;
};

struct RouteCommand {
    std::optional<Waypoint> target;
    double speed;
    double steering;
    double tracking_error = 0;
    double speed_error = 0;
};

class RouteController {
public:
    RouteController(std::vector<Waypoint> route, RoutePosition origin, double dt,
                    bool until_destination, double speed, double steering);

    RouteCommand update(const std::array<double, 3>& position, RoutePosition velocity,
                        double heading, double time);
    const std::optional<RouteDestination>& destination() const noexcept { return destination_; }

private:
    std::optional<Waypoint> target_at(double time) const;
    std::vector<Waypoint> route_;
    RoutePosition origin_;
    double dt_;
    double speed_;
    double steering_;
    std::vector<double> times_;
    std::optional<RouteDestination> destination_;
};

} // namespace fusionforce
