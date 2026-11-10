#include "fusionforce/truck_step.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace {

struct TestDirectory {
    const std::filesystem::path path = std::filesystem::temp_directory_path()
        / ("fusionforce-truck-step-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));

    TestDirectory()
    {
        if (!std::filesystem::create_directory(path)) {
            throw std::runtime_error("Cannot create truck test directory");
        }
    }

    ~TestDirectory()
    {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
};

void check(bool condition, const char* label)
{
    if (!condition) {
        throw std::runtime_error(label);
    }
}

void near(double actual, double expected, double tolerance, const char* label)
{
    check(std::abs(actual - expected) <= tolerance, label);
}

} // namespace

int main()
{
    using namespace fusionforce;

    const TestDirectory scratch;
    const auto source = scratch.path / "flat.ply";
    {
        std::ofstream ply(source);
        ply << "ply\nformat ascii 1.0\nelement vertex " << 71 * 21
            << "\nproperty double x\nproperty double y\nproperty double z\nproperty int label\nend_header\n";
        for (int y = -10; y <= 10; ++y) {
            for (int x = -35; x <= 35; ++x) {
                ply << 200 + x * .2 << ' ' << -100 + y * .2 << " 1000 5\n";
            }
        }
        check(static_cast<bool>(ply), "write synthetic PLY");
    }
    CloudOptions options;
    options.cache_dir = scratch.path / "cache";
    options.label_mapping = CloudLabels{{"5", "barro"}};
    const auto cloud = CloudStore::open(source, options);

    TruckGeometry truck{};
    truck.mass = 200;
    truck.moments_of_inertia = {30, 25, 30};
    truck.corners = {{{.5f, .5f, 0}, {.5f, -.5f, 0}, {-.5f, .5f, 0}, {-.5f, -.5f, 0}}};
    truck.corner_steering_factors = {1, 1, 0, 0};
    rigid::State state;
    state.position[2] = -.02f;
    const CloudXyz origin{200.13, -100.08, 1000};
    TruckStepper stepper(origin);
    TruckTerrain terrain;
    const auto first = stepper.step(cloud, truck, state, {}, 1, 0, true, terrain);
    check(first.global_points.size() == 4, "contact geometry has four corners");
    check(first.contact_materials.at(0).terrain_class == "barro", "labeled terrain class");
    near(first.global_points.at(0)[0], 200.63, 1e-6, "global contact x");
    near(first.global_points.at(0)[2], 999.98, 1e-6, "global contact z");
    near(first.global_cog[1], -100.08, 1e-6, "global center of gravity");
    const double weight = .5 * (1 + std::tanh(2 * std::sqrt(3.0)));
    const double spring = 1e5 * .02 * weight / 4;
    near(first.physics.contacts.at(0).spring[2], spring, 1e-3, "labeled terrain stiffness");
    near(first.physics.contacts.at(0).friction[0], .25 * spring * std::tanh(1.0), 1e-3, "labeled friction");

    state.position[0] = 4.8f;
    const auto second = stepper.step(cloud, truck, state, {}, 1, 0, true, terrain);
    near(second.physics.contacts.at(0).spring[2], spring, 1e-3, "spring after rolling patch update");
    near(second.physics.state.position[0] - state.position[0], first.physics.state.position[0],
         1e-6, "restore original XY reference frame");

    terrain.zones.push_back({{205, -100, 1000}, 2, "grava"});
    terrain.zones.push_back({{205, -100, 1001}, 2, "roca"});
    terrain.zones.push_back({{205, -100, 1005}, 2, "tierra"});
    const auto overridden = stepper.step(cloud, truck, state, {}, 1, 0, true, terrain);
    check(overridden.contact_materials.at(0).terrain_class == "roca", "zone terrain class");
    near(overridden.physics.contacts.at(0).spring[2], 30 * spring, .02, "last matching spherical zone");

    state.position[1] = 7;
    bool rejected = false;
    try {
        stepper.step(cloud, truck, state, {}, 1, 0, true, terrain);
    } catch (const TruckCoverageError&) {
        rejected = true;
    }
    check(rejected, "uncovered footprint rejected");
}
