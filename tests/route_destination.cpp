#include "fusionforce/route_destination.hpp"

#include <cmath>
#include <stdexcept>

namespace {

void check(bool condition)
{
    if (!condition) {
        throw std::runtime_error("route destination check failed");
    }
}

} // namespace

int main()
{
    using fusionforce::RouteDestination;
    RouteDestination route({{0, 0, 2, 1, 1}, {2, 0, 5, 1, 1}, {4, 0, 8, 1, 1}}, {0, 0});
    check(route.estimated_duration() == 4);
    check(route.time_limit() == 32);
    check(route.observe({0, 0}, 0)->x == 2);
    check(route.observe({4, 0}, 1)->x == 2);
    check(route.remaining() == 4);
    check(route.observe({1.5, 0}, 2)->x == 4);
    check(route.last_improvement() == 2);
    check(route.observe({1.51, 0}, 3)->x == 4);
    check(route.last_improvement() == 2);
    check(route.observe({1.6, 0}, 4)->x == 4);
    check(route.last_improvement() == 4);
    check(route.observe({3.5, 0}, 5) == nullptr);
    check(route.reached() && route.remaining() == 0);

    RouteDestination far({{1000000, 0, 0, 1, 0}}, {0, 0});
    check(far.time_limit() == 1800);
    RouteDestination slow({{1, 0, 0, 0, 0}}, {0, 0});
    check(slow.estimated_duration() == 4);
    RouteDestination fast({{1, 0, 0, 100, 0}}, {0, 0});
    check(std::abs(fast.estimated_duration() - 0.05) < 1e-12);

    const std::vector<fusionforce::Waypoint> waypoints{{0, 0, 2, 1, 0}, {2, 0, 4, 1, 0}};
    fusionforce::RouteController timed(waypoints, {0, 0}, 0.01, false, 0, 0);
    const auto timed_command = timed.update({0, 0, 3}, {0, 0}, 0, 1);
    check(timed_command.target->x == 1 && timed_command.target->z == 3);
    check(timed_command.tracking_error == 1 && timed_command.speed_error == 1);
    check(std::abs(timed_command.speed - 0.04) < 1e-12);
    fusionforce::RouteController destination(waypoints, {0, 0}, 0.01, true, 0, 0);
    const auto destination_command = destination.update({0, 0, 100}, {0, 0}, 0, 0);
    check(destination_command.target->x == 2 && destination_command.tracking_error == 0);
    check(std::abs(destination_command.speed - 0.02) < 1e-12);
    check(!destination.update({1.5, 0, 100}, {0, 0}, 0, 1).target);
    check(destination.destination()->reached());
}
