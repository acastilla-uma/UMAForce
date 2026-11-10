#include "fusionforce/simulation.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <source_location>
#include <sstream>
#include <stdexcept>

namespace {

struct Scratch {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("fusionforce-simulation-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));

    Scratch() {
        if (!std::filesystem::create_directory(path)) {
            throw std::runtime_error("Cannot create simulation test directory");
        }
    }
    ~Scratch() {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
};

void check(bool condition, std::source_location location = std::source_location::current())
{
    if (!condition) {
        throw std::runtime_error("simulation check failed on line " + std::to_string(location.line()));
    }
}

} // namespace

int main()
{
    using namespace fusionforce;
    const Scratch scratch;
    const auto source = scratch.path / "flat.ply";
    {
        std::ofstream ply(source);
        ply << "ply\nformat ascii 1.0\nelement vertex 121\nproperty float x\n"
               "property float y\nproperty float z\nend_header\n";
        for (int y = -5; y <= 5; ++y) {
            for (int x = -5; x <= 5; ++x) {
                ply << x * .2 << ' ' << y * .2 << " 0\n";
            }
        }
        check(static_cast<bool>(ply));
    }
    CloudOptions options;
    options.cache_dir = scratch.path / "cache";
    const auto cloud = CloudStore::open(source, options);

    TruckGeometry truck{};
    truck.mass = 200;
    truck.moments_of_inertia = {30, 25, 30};
    truck.center_of_gravity = {0, 0, .5f};
    truck.corners = {{{.5f, .5f, 0}, {.5f, -.5f, 0}, {-.5f, .5f, 0}, {-.5f, -.5f, 0}}};
    truck.wheel_centers.assign(truck.corners.begin(), truck.corners.end());
    SimulationInput input;
    input.origin[2] = .2;
    input.duration = .08;
    input.lightweight = true;
    std::vector<double> fractions;
    std::ostringstream csv;
    const auto manual = simulate_truck(cloud, truck, {}, input,
        [&](SimulationProgress value) { fractions.push_back(value.fraction); }, &csv);
    check(manual.status == "complete" && manual.samples == 9 && manual.frames.size() == 3);
    check(manual.frames.front().global_position[2] == .2);
    check(manual.frames.back().global_position[2] < .2);
    check(manual.frames.back().stability.contains("ltr"));
    check(fractions.front() == 0 && fractions.back() == 1);
    check(std::count(fractions.begin(), fractions.end(), 1.0) == 1);
    const auto csv_text = csv.str();
    check(std::count(csv_text.begin(), csv_text.end(), '\n') == 46);
    check(csv_text.find("contact,front_left") != std::string::npos);

    input.until_destination = true;
    input.route = {{0, 0, 2, 1, 1}, {.5, 0, 3, 1, 1}};
    const auto arrived = simulate_truck(cloud, truck, {}, input);
    check(arrived.destination_reached == true && arrived.distance_remaining == 0);
    check(arrived.samples == 1 && arrived.frames.size() == 1 && !arrived.stopped_reason);
}
