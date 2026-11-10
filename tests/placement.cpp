#include "fusionforce/placement.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace {

using namespace fusionforce;

void check(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(double actual, double expected)
{
    check(std::abs(actual - expected) < 1e-6, "unexpected placement coordinate");
}

PlacementGeometry geometry()
{
    PlacementGeometry model;
    model.radius = 0.6;
    model.width = 0.4;
    model.vehicle_id = "MAN_5t";
    for (double x : {2.0, -2.0}) {
        for (double y : {-1.0, 1.0}) {
            model.centers.push_back({x, y, 0});
        }
    }
    for (double y : {-0.2, 0.2}) {
        for (int index = 0; index < 24; ++index) {
            const double angle = index * (3.14159265358979323846 / 12);
            model.ring.push_back({0.6 * std::sin(angle), y, 0.6 * std::cos(angle)});
        }
    }
    for (double x : {-3.0, 3.0}) {
        for (double y : {-1.2, 1.2}) {
            model.body.push_back({x, y, 0.5});
        }
    }
    return model;
}

PlacementRegion region(std::vector<PlacementPoint> points)
{
    return [points = std::move(points)](const PlacementBounds& bounds) {
        std::vector<PlacementPoint> selected;
        for (const auto& point : points) {
            if (point[0] >= bounds[0] && point[0] <= bounds[2] &&
                point[1] >= bounds[1] && point[1] <= bounds[3]) {
                selected.push_back(point);
            }
        }
        return selected;
    };
}

template<class Operation>
void invalid(Operation operation, std::string_view expected)
{
    try {
        operation();
    } catch (const std::invalid_argument& error) {
        check(std::string_view(error.what()).find(expected) != std::string_view::npos,
              "unexpected placement error");
        return;
    }
    throw std::runtime_error("invalid placement was accepted");
}

} // namespace

int main()
{
    const auto model = geometry();
    const auto layered = region({{0, 0, 0}, {0, 0, 2}});
    const auto flat = place_vehicle(layered, model, {0, 0}, 0);
    near(flat.position[2], 2.6 + drop_clearance);
    check(flat.roll == 0 && flat.pitch == 0 && flat.yaw == 0, "expected a level drop seed");
    check(flat.wheel_centers == model.centers, "wheel geometry was changed");
    check(flat.vehicle == "MAN_5t", "incorrect vehicle metadata");
    for (const auto& point : flat.wheel_contacts) {
        near(point[2], 2 + drop_clearance);
    }

    const auto sparse = region({{0, 0, 0}, {0, 2.5, 1}, {2.5, 0, 12}});
    const auto rotated = place_vehicle(sparse, model, {0, 0}, 450);
    near(rotated.position[2], 1.6 + drop_clearance);
    near(rotated.orientation[0], std::sqrt(0.5));
    near(rotated.orientation[3], std::sqrt(0.5));
    near(rotated.wheel_contacts.front()[0], 1);
    near(rotated.wheel_contacts.front()[1], 2);
    check(rotated.yaw == 90, "yaw was not normalized");

    const auto edge = place_vehicle(region({{500000, 4100000, 300}}), model,
                                    {500000, 4100000}, 30);
    near(edge.position[0], 500000);
    near(edge.position[1], 4100000);
    near(edge.position[2], 300.6 + drop_clearance);
    check(place_vehicle(layered, model, {0, 0}, -180).yaw == -180, "incorrect yaw endpoint");

    invalid([&] { place_vehicle(region({{1.3, 0, 0}}), model, {0, 0}, 0); }, "no hay puntos");
    invalid([&] { place_vehicle(layered, model, {100, 0}, 0); }, "no hay puntos");
    invalid([&] { place_vehicle(layered, model, {std::nan(""), 0}, 0); }, "XY");
    invalid([&] { place_vehicle(layered, model, {0, 0}, std::numeric_limits<double>::infinity()); },
            "ngulo");
    auto broken = model;
    broken.ring.clear();
    invalid([&] { place_vehicle(layered, broken, {0, 0}, 0); }, "veh");
}
