#include "fusionforce/viewer_inputs.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <stdexcept>
#include <string_view>

namespace {

using nlohmann::json;

void check(bool condition, const char* error)
{
    if (!condition) {
        throw std::runtime_error(error);
    }
}

template<class Operation>
void rejects(Operation operation, std::string_view message)
{
    try {
        operation();
    } catch (const std::invalid_argument& error) {
        check(std::string_view(error.what()).find(message) != std::string_view::npos,
              "unexpected viewer validation error");
        return;
    }
    throw std::runtime_error("invalid request was accepted");
}

} // namespace

int main()
{
    using namespace fusionforce;
    const CloudXyz center{500000, 4100000, 300};
    const auto defaults = simulation_input(json::object(), center);
    check(defaults.origin == CloudXyz{500000, 4100000, 0}, "unexpected pose defaults");
    check(defaults.dt == .01 && defaults.duration == 0, "unexpected simulation defaults");

    const auto request = json::parse(R"({
        "x": " +500000.5 ", "z": 301, "vx": "2.5", "wy": -1, "yaw": 90,
        "steering": 39, "lightweight_physics": true, "until_destination": true,
        "duration": -999, "route": [{"x": 500000, "y": 4100000}, {"x": 500002, "speed": 3}]
    })");
    const auto input = simulation_input(request, center);
    check(input.origin == CloudXyz{500000.5, 4100000, 301}, "pose was not parsed");
    check(input.initial.velocity[0] == 2.5f && input.initial.angular_velocity[1] == -1,
          "initial velocities were not parsed");
    check(std::abs(input.initial.orientation[0] - std::sqrt(.5)) < 1e-7 &&
          std::abs(input.initial.orientation[3] - std::sqrt(.5)) < 1e-7,
          "quaternion convention changed");
    check(input.steering > .68 && input.steering < .69, "steering was not converted to radians");
    check(input.lightweight && input.until_destination && input.duration == 0, "unexpected run mode");
    check(input.route.size() == 2 && input.route[0].z == 301 && input.route[1].speed == 3 &&
          input.route[0].acceleration == 1, "unexpected waypoint defaults");

    rejects([&] { simulation_input(json{{"disable_csv", 1}}, center); }, "booleana");
    rejects([&] { simulation_input(json{{"until_destination", true}}, center); }, "dos puntos");
    rejects([&] { simulation_input(json{{"route", "invalid"}}, center); }, "Ruta");
    rejects([&] { simulation_input(json{{"steering", 39.01}}, center); }, "steering");
    rejects([&] { simulation_input(json{{"x", "10x"}}, center); }, "x");
    rejects([&] { simulation_input(json::object(), {2e12, 0, 0}); }, "x");

    const auto terrain_request = json::parse(R"({
        "friction": 1, "stiffness": 9000,
        "terrain": {
            "classes": {"desconocido": {"friction": 1.5}, "barro": {"stiffness": 25000}},
            "zones": [{"class": "barro", "center": [500000, 4100000, 300], "radius": 2}]
        }
    })");
    const auto terrain = truck_terrain(terrain_request);
    check(terrain.classes.at("desconocido").friction_lon == 1.5f &&
          terrain.classes.at("desconocido").stiffness == 9000,
          "class override discarded the root material");
    check(terrain.classes.at("barro").stiffness == 25000 &&
          terrain.classes.at("barro").friction_lat == .25f, "class defaults changed");
    check(terrain.zones.size() == 1 && terrain.zones[0].center == center &&
          terrain.zones[0].radius == 2 && terrain.zones[0].terrain_class == "barro",
          "terrain zone was not parsed");
    rejects([] { truck_terrain(json::parse(R"({"terrain":{"classes":{"agua":{}}}})")); },
            "agua");
    rejects([] {
        truck_terrain(json::parse(R"({"terrain":{"zones":[{"class":"barro","center":[0,1]}]}})"));
    }, "Zona");
}
