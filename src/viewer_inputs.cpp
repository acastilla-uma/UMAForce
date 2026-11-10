#include "fusionforce/viewer_inputs.hpp"

#include "fusionforce/settling.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <iterator>
#include <numbers>
#include <stdexcept>
#include <string_view>

namespace fusionforce {
namespace {

using Json = nlohmann::json;
constexpr double degrees_to_radians = std::numbers::pi / 180;
constexpr std::size_t max_route_points = 10'000;
constexpr std::size_t max_terrain_zones = 10'000;
constexpr double coordinate_limit = 1e12;

double numeric(const Json& value, const std::string& error)
{
    if (value.is_number()) {
        return value.get<double>();
    }
    if (value.is_boolean()) {
        return value.get<bool>() ? 1 : 0;
    }
    if (value.is_string()) {
        const auto& text = value.get_ref<const std::string&>();
        constexpr std::string_view whitespace = " \t\n\r\f\v";
        const auto first = text.find_first_not_of(whitespace);
        if (first != std::string::npos) {
            const auto last = text.find_last_not_of(whitespace);
            const char* begin = text.data() + first;
            const char* end = text.data() + last + 1;
            if (*begin == '+' && begin + 1 < end && begin[1] != '-') {
                ++begin;
            }
            double number;
            const auto parsed = std::from_chars(begin, end, number);
            if (parsed.ec == std::errc{} && parsed.ptr == end) {
                return number;
            }
        }
    }
    throw std::invalid_argument(error);
}

double number(const Json& object, const char* key, double fallback, double low, double high)
{
    const auto value = object.find(key);
    const auto error = std::string("Valor inválido: ") + key;
    const double result = value == object.end() ? fallback : numeric(*value, error);
    if (!std::isfinite(result) || result < low || result > high) {
        throw std::invalid_argument(error);
    }
    return result;
}

bool boolean(const Json& object, const char* key, const char* error)
{
    const auto value = object.find(key);
    if (value == object.end()) {
        return false;
    }
    if (!value->is_boolean()) {
        throw std::invalid_argument(error);
    }
    return value->get<bool>();
}

bool lightweight_mode(const Json& request)
{
    return boolean(request, "lightweight_physics", "La opción de física ligera debe ser booleana");
}

rigid::Quaternion quaternion(double roll, double pitch, double yaw)
{
    const double r = roll * degrees_to_radians / 2;
    const double p = pitch * degrees_to_radians / 2;
    const double y = yaw * degrees_to_radians / 2;
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    return {
        static_cast<float>(cr*cp*cy + sr*sp*sy),
        static_cast<float>(sr*cp*cy - cr*sp*sy),
        static_cast<float>(cr*sp*cy + sr*cp*sy),
        static_cast<float>(cr*cp*sy - sr*sp*cy),
    };
}

rigid::Vec3 vector(const Json& request, const char* x, const char* y, const char* z)
{
    return {
        static_cast<float>(number(request, x, 0, -100, 100)),
        static_cast<float>(number(request, y, 0, -100, 100)),
        static_cast<float>(number(request, z, 0, -100, 100)),
    };
}

CloudXyz zone_center(const Json& zone)
{
    constexpr const char* error = "Zona de terreno inválida";
    const auto center = zone.find("center");
    if (center == zone.end() || !center->is_array() || center->size() != 3) {
        throw std::invalid_argument(error);
    }
    CloudXyz xyz{};
    for (std::size_t i = 0; i < xyz.size(); ++i) {
        xyz[i] = numeric((*center)[i], error);
        if (!std::isfinite(xyz[i])) {
            throw std::invalid_argument(error);
        }
    }
    return xyz;
}

rigid::Material material(const Json& request, double stiffness, double friction)
{
    const auto mu = static_cast<float>(number(request, "friction", friction, 0, 3));
    const auto spring = static_cast<float>(number(request, "stiffness", stiffness, 1000, 1e8));
    return {spring, mu, mu};
}

PlacementGeometry placement_geometry(const TruckGeometry& truck)
{
    PlacementGeometry result;
    const auto convert = [](const TruckPoint& p) -> PlacementPoint {
        return {p[0], p[1], p[2]};
    };
    std::transform(truck.wheel_centers.begin(), truck.wheel_centers.end(),
                   std::back_inserter(result.centers), convert);
    std::transform(truck.ring.begin(), truck.ring.end(), std::back_inserter(result.ring), convert);
    std::transform(truck.body.begin(), truck.body.end(), std::back_inserter(result.body), convert);
    result.radius = truck.radius;
    result.width = truck.width;
    result.vehicle_id = truck.vehicle_id;
    return result;
}

Json placement_json(const PlacementSeed& pose)
{
    return {
        {"position", pose.position}, {"orientation", pose.orientation},
        {"roll", pose.roll}, {"pitch", pose.pitch}, {"yaw", pose.yaw},
        {"wheel_contacts", pose.wheel_contacts}, {"wheel_centers", pose.wheel_centers},
        {"tire_radius", pose.tire_radius}, {"tire_width", pose.tire_width},
        {"support_clearance", pose.support_clearance}, {"vehicle", pose.vehicle},
    };
}

} // namespace

SimulationInput simulation_input(const Json& request, CloudXyz center)
{
    if (!request.is_object()) {
        throw std::invalid_argument("Parámetros de simulación inválidos");
    }
    boolean(request, "disable_csv", "La opción de registro CSV debe ser booleana");
    SimulationInput input;
    input.lightweight = lightweight_mode(request);
    input.dt = number(request, "dt", .01, .0005, .01);
    input.until_destination = boolean(request, "until_destination",
                                      "La opción de llegada al destino debe ser booleana");
    input.duration = input.until_destination ? 0 : number(request, "duration", 0, 0, 60);
    input.origin = {
        number(request, "x", center[0], -coordinate_limit, coordinate_limit),
        number(request, "y", center[1], -coordinate_limit, coordinate_limit),
        number(request, "z", 0, -coordinate_limit, coordinate_limit),
    };
    const double roll = number(request, "roll", 0, -180, 180);
    const double pitch = number(request, "pitch", 0, -180, 180);
    const double yaw = number(request, "yaw", 0, -180, 180);
    input.initial.orientation = quaternion(roll, pitch, yaw);
    input.initial.velocity = vector(request, "vx", "vy", "vz");
    input.initial.angular_velocity = vector(request, "wx", "wy", "wz");
    const auto route = request.find("route");
    if (route != request.end()) {
        if (!route->is_array() || route->size() > max_route_points) {
            throw std::invalid_argument("Ruta inválida");
        }
        input.route.reserve(route->size());
        for (const auto& point : *route) {
            if (!point.is_object()) {
                throw std::invalid_argument("Ruta inválida");
            }
            input.route.push_back({
                number(point, "x", 0, -coordinate_limit, coordinate_limit),
                number(point, "y", 0, -coordinate_limit, coordinate_limit),
                number(point, "z", input.origin[2], -coordinate_limit, coordinate_limit),
                number(point, "speed", 0, 0, 100),
                number(point, "acceleration", 1, -20, 20),
            });
        }
    }
    if (input.until_destination && input.route.size() < 2) {
        throw std::invalid_argument("Se requieren al menos dos puntos para llegar al destino");
    }
    input.speed = number(request, "speed", 0, -20, 20);
    input.steering = number(request, "steering", 0, -39, 39) * degrees_to_radians;
    return input;
}

TruckTerrain truck_terrain(const Json& request)
{
    TruckTerrain result;
    result.classes.at("desconocido") = material(request, 1e6, .7);
    const auto terrain = request.find("terrain");
    if (terrain == request.end() || terrain->is_null() ||
        (terrain->is_boolean() && !terrain->get<bool>()) ||
        (terrain->is_number() && terrain->get<double>() == 0) ||
        ((terrain->is_object() || terrain->is_array()) && terrain->empty()) ||
        (terrain->is_string() && terrain->get_ref<const std::string&>().empty())) {
        return result;
    }
    if (!terrain->is_object()) {
        throw std::invalid_argument("Terreno inválido");
    }
    if (const auto classes = terrain->find("classes"); classes != terrain->end()) {
        if (!classes->is_object()) {
            throw std::invalid_argument("Clases de terreno inválidas");
        }
        for (const auto& [name, values] : classes->items()) {
            const auto existing = result.classes.find(name);
            if (existing == result.classes.end()) {
                throw std::invalid_argument("Clase de terreno desconocida: " + name);
            }
            if (!values.is_object()) {
                throw std::invalid_argument("Clase de terreno inválida: " + name);
            }
            const auto original = existing->second;
            existing->second = material(values, original.stiffness, original.friction_lon);
        }
    }
    if (const auto zones = terrain->find("zones"); zones != terrain->end()) {
        if (!zones->is_array() || zones->size() > max_terrain_zones) {
            throw std::invalid_argument("Demasiadas zonas");
        }
        result.zones.reserve(zones->size());
        for (const auto& zone : *zones) {
            const auto center = zone_center(zone);
            const auto terrain_class = zone.find("class");
            if (terrain_class == zone.end() || !terrain_class->is_string() ||
                result.classes.count(terrain_class->get_ref<const std::string&>()) == 0) {
                throw std::invalid_argument("Zona de terreno inválida");
            }
            result.zones.push_back({center, number(zone, "radius", 1, .001, 1e6),
                                    terrain_class->get<std::string>()});
        }
    }
    return result;
}

Json place_from_request(const Json& request, const CloudStore& cloud, const TruckGeometry& truck)
{
    if (!request.is_object()) {
        throw std::invalid_argument("Pose inválida");
    }
    const auto purpose = request.find("purpose");
    if (purpose != request.end() && *purpose != "initial" && *purpose != "waypoint") {
        throw std::invalid_argument("Tipo de colocación inválido");
    }
    const bool lightweight = lightweight_mode(request);
    constexpr const char* position_error = "Selecciona una posición XY válida sobre el terreno";
    const auto x = request.find("x");
    const auto y = request.find("y");
    if (x == request.end() || y == request.end()) {
        throw std::invalid_argument(position_error);
    }
    const PlacementXY xy{numeric(*x, position_error), numeric(*y, position_error)};
    const auto yaw_value = request.find("yaw");
    constexpr const char* yaw_error = "La orientación debe ser un ángulo válido";
    if (yaw_value != request.end() && yaw_value->is_boolean()) {
        throw std::invalid_argument(yaw_error);
    }
    const double yaw = yaw_value == request.end() ? 0 : numeric(*yaw_value, yaw_error);
    const auto seed = place_vehicle([&cloud](const PlacementBounds& bounds) {
        const auto points = cloud.region_xy(bounds);
        std::vector<PlacementPoint> xyz;
        xyz.reserve(points.size());
        for (const auto& point : points) {
            xyz.push_back(point.xyz);
        }
        return xyz;
    }, placement_geometry(truck), xy, yaw);
    if (purpose != request.end() && *purpose == "waypoint") {
        return placement_json(seed);
    }
    for (std::size_t i = 0; i < seed.position.size(); ++i) {
        if (!std::isfinite(seed.position[i]) || std::abs(seed.position[i]) > coordinate_limit) {
            throw std::invalid_argument(std::string("Valor inválido: ") + "xyz"[i]);
        }
    }
    const auto result = settle_vehicle(seed, cloud, truck, lightweight, truck_terrain(request));
    auto response = placement_json(result.pose);
    response["grounded"] = result.grounded;
    response["settled"] = result.settled;
    response["settling_time"] = result.settling_time;
    return response;
}

} // namespace fusionforce
