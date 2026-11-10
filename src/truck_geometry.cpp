#include "fusionforce/truck_geometry.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace fusionforce {
namespace {

using nlohmann::json;

std::ifstream open_asset(const std::filesystem::path& path)
{
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("No se pudo abrir el recurso: " + path.string());
    }
    return file;
}

json read_asset(const std::filesystem::path& path)
{
    auto file = open_asset(path);
    return json::parse(file);
}

std::vector<TruckPoint> read_body(const std::filesystem::path& path)
{
    auto file = open_asset(path);
    std::vector<std::array<double, 3>> vertices;
    std::string line;
    while (std::getline(file, line)) {
        if (line.rfind("v ", 0) != 0) {
            continue;
        }
        std::istringstream values(line.substr(2));
        std::array<double, 3> vertex{};
        if (!(values >> vertex[0] >> vertex[1] >> vertex[2])) {
            throw std::runtime_error("Vértice OBJ inválido: " + path.string());
        }
        vertices.push_back(vertex);
    }
    const std::size_t stride = std::max<std::size_t>(1, vertices.size() / 250);
    std::sort(vertices.begin(), vertices.end());
    vertices.erase(std::unique(vertices.begin(), vertices.end()), vertices.end());
    std::vector<TruckPoint> points;
    for (std::size_t i = 0; i < vertices.size(); i += stride) {
        const auto& vertex = vertices[i];
        points.push_back({float(vertex[0]), float(vertex[1]), float(vertex[2])});
    }
    return points;
}

} // namespace

TruckGeometry load_truck_geometry(const std::filesystem::path& root, const std::string& id)
{
    std::string drivetrain;
    if (id == "MAN_5t") {
        drivetrain = "4WD";
    } else if (id == "MAN_7t") {
        drivetrain = "6WD";
    } else if (id == "MAN_10t") {
        drivetrain = "8WD";
    } else {
        throw std::invalid_argument("Vehículo no compatible");
    }

    const auto chassis = read_asset(root / "chassis" / (id + "_Chassis.json"));
    const auto vehicle = read_asset(root / "vehicle" / (id + "_Vehicle_" + drivetrain + ".json"));
    const auto tire = read_asset(root / "tire" / "MAN_RigidMeshTire.json");
    const auto& body = chassis.at("Components").at(0);
    TruckGeometry truck{};
    truck.vehicle_id = id;
    truck.mass = body.at("Mass").get<double>();
    truck.center_of_gravity = body.at("Centroidal Frame").at("Location").get<TruckPoint>();
    truck.moments_of_inertia = body.at("Moments of Inertia").get<TruckPoint>();
    truck.radius = tire.value("Radius", 0.645);
    truck.width = tire.value("Width", 0.385);

    for (const auto& axle : vehicle.at("Axles")) {
        const auto name = axle.at("Suspension Input File").get<std::string>();
        if (name.rfind("MAN/", 0) != 0) {
            throw std::runtime_error("Recurso de suspensión MAN inválido");
        }
        const auto suspension = read_asset(root / name.substr(4));
        const auto location = axle.at("Suspension Location").get<std::array<double, 3>>();
        const auto spindle = suspension.at("Spindle").at("COM").get<std::array<double, 3>>();
        const TruckPoint center = {float(location[0] + spindle[0]), float(location[1] + spindle[1]),
                                   float(location[2] + spindle[2])};
        truck.wheel_centers.push_back(center);
        truck.wheel_centers.push_back({center[0], -center[1], center[2]});
        const float steering = axle.contains("Steering Index")
            ? (axle.at("Steering Index") == 0 ? 1.0f : 0.7f) : 0.0f;
        truck.steering_factors.insert(truck.steering_factors.end(), 2, steering);
    }
    if (truck.wheel_centers.empty()) {
        throw std::runtime_error("El vehículo no contiene ejes");
    }
    const double turn = 2 * std::acos(-1.0);
    for (const double y : {-truck.width / 2, truck.width / 2}) {
        for (int i = 0; i < 24; ++i) {
            const double angle = (turn / 24) * i;
            truck.ring.push_back({float(truck.radius * std::sin(angle)), float(y),
                                  float(truck.radius * std::cos(angle))});
        }
    }
    truck.body = read_body(root / "meshes" / (id + "_chassis_col.obj"));

    TruckPoint low = truck.wheel_centers.front();
    TruckPoint high = low;
    const auto include = [&](const TruckPoint& point) {
        for (std::size_t i = 0; i < point.size(); ++i) {
            low[i] = std::min(low[i], point[i]);
            high[i] = std::max(high[i], point[i]);
        }
    };
    for (const auto& center : truck.wheel_centers) {
        for (const auto& point : truck.ring) {
            include({center[0] + point[0], center[1] + point[1], center[2] + point[2]});
        }
    }
    for (const auto& point : truck.body) {
        include(point);
    }
    truck.corners = {{{high[0], high[1], low[2]}, {high[0], low[1], low[2]},
                      {low[0], high[1], low[2]}, {low[0], low[1], low[2]}}};
    const auto by_x = [](const TruckPoint& a, const TruckPoint& b) { return a[0] < b[0]; };
    const auto front = std::max_element(truck.wheel_centers.begin(), truck.wheel_centers.end(), by_x);
    const auto rear = std::min_element(truck.wheel_centers.begin(), truck.wheel_centers.end(), by_x);
    const float front_steering = truck.steering_factors[front - truck.wheel_centers.begin()];
    const float rear_steering = truck.steering_factors[rear - truck.wheel_centers.begin()];
    truck.corner_steering_factors = {front_steering, front_steering, rear_steering, rear_steering};
    return truck;
}

TruckContactGeometry transform_truck(const TruckGeometry& truck, const rigid::State& state,
                                     float speed, float steering, bool lightweight)
{
    const auto rotation = rigid::rotation_matrix(state.orientation);
    const auto position = [&](const TruckPoint& local) {
        auto world = rigid::transform(rotation, local);
        for (std::size_t i = 0; i < world.size(); ++i) {
            world[i] += state.position[i];
        }
        return world;
    };
    rigid::Mat3 inertia{};
    for (std::size_t i = 0; i < inertia.size(); ++i) {
        inertia[i][i] = truck.moments_of_inertia[i];
    }
    TruckContactGeometry result{position(truck.center_of_gravity),
                                rigid::rotate_inertia(inertia, rotation), {}};
    const auto drive = [&](float cosine, float sine) {
        return rigid::transform(rotation, {cosine * speed, sine * speed, 0});
    };
    if (lightweight) {
        for (std::size_t i = 0; i < truck.corners.size(); ++i) {
            const float angle = truck.corner_steering_factors[i] * steering;
            result.contacts.push_back({position(truck.corners[i]), drive(std::cos(angle), std::sin(angle)), {}});
        }
        return result;
    }
    result.contacts.reserve(truck.wheel_centers.size() * truck.ring.size() + truck.body.size());
    for (std::size_t i = 0; i < truck.wheel_centers.size(); ++i) {
        const float angle = truck.steering_factors[i] * steering;
        const float cosine = std::cos(angle);
        const float sine = std::sin(angle);
        const auto thrust = drive(cosine, sine);
        const auto& center = truck.wheel_centers[i];
        for (const auto& point : truck.ring) {
            const TruckPoint rotated = {cosine * point[0] - sine * point[1] + center[0],
                                        sine * point[0] + cosine * point[1] + center[1],
                                        point[2] + center[2]};
            result.contacts.push_back({position(rotated), thrust, {}});
        }
    }
    for (const auto& point : truck.body) {
        result.contacts.push_back({position(point), {}, {}});
    }
    return result;
}

} // namespace fusionforce
