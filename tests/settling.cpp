#include "fusionforce/settling.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace {

using namespace fusionforce;

void check(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

struct TestDirectory {
    const std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("fusionforce-settling-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));

    TestDirectory()
    {
        check(std::filesystem::create_directory(path), "cannot create settling test directory");
    }

    ~TestDirectory()
    {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
};

CloudStore write_cloud(const TestDirectory& scratch, const char* name, int half_side)
{
    const auto path = scratch.path / name;
    std::ofstream ply(path);
    const int side = 2 * half_side + 1;
    ply << "ply\nformat ascii 1.0\nelement vertex " << side * side
        << "\nproperty double x\nproperty double y\nproperty double z\nend_header\n";
    ply.precision(12);
    for (int y = -half_side; y <= half_side; ++y) {
        for (int x = -half_side; x <= half_side; ++x) {
            ply << 500000.0 + 0.2*x << ' ' << 4100000.0 + 0.2*y << " 300\n";
        }
    }
    ply.close();
    check(static_cast<bool>(ply), "could not write test PLY");
    CloudOptions options;
    options.cache_dir = scratch.path / "cache";
    return CloudStore::open(path, options);
}

PlacementSeed seed_for(const CloudStore& cloud, const TruckGeometry& truck)
{
    PlacementGeometry geometry;
    const auto copy_point = [](const TruckPoint& point) {
        return PlacementPoint{point[0], point[1], point[2]};
    };
    std::transform(truck.wheel_centers.begin(), truck.wheel_centers.end(),
                   std::back_inserter(geometry.centers), copy_point);
    std::transform(truck.ring.begin(), truck.ring.end(), std::back_inserter(geometry.ring), copy_point);
    std::transform(truck.body.begin(), truck.body.end(), std::back_inserter(geometry.body), copy_point);
    geometry.radius = truck.radius;
    geometry.width = truck.width;
    geometry.vehicle_id = truck.vehicle_id;
    return place_vehicle([&](const PlacementBounds& bounds) {
        const auto points = cloud.region_xy(bounds);
        std::vector<PlacementPoint> xyz;
        xyz.reserve(points.size());
        for (const auto& point : points) {
            xyz.push_back(point.xyz);
        }
        return xyz;
    }, geometry, {500000.0, 4100000.0}, 35);
}

} // namespace

int main()
{
    const TestDirectory scratch;
    const auto flat = write_cloud(scratch, "flat.ply", 55);
    const auto sparse = write_cloud(scratch, "sparse.ply", 1);
    const TruckTerrain terrain;
    struct Baseline {
        const char* vehicle;
        bool lightweight;
        double time;
        PlacementPoint position;
        std::array<double, 4> orientation;
    };
    // Python Simulation.place on the same survey-coordinate PLY and heading.
    // Compare the contact step exactly and allow float32 reduction differences.
    const Baseline baselines[] = {
        {"MAN_5t", false, .41, {500000.01144536, 4100000.00801526, 300.38355203},
            {.95368195, -.00257706, .00817393, .30069482}},
        {"MAN_5t", true, .35, {500000.00601554, 4100000.00421281, 300.49572049},
            {.95371103, -.00106329, .00337239, .30070394}},
        {"MAN_7t", false, .49, {500000.03539677, 4100000.02479993, 300.29201664},
            {.95346880, -.00685763, .02174947, .30062810}},
        {"MAN_7t", true, .37, {500000.00515237, 4100000.00360803, 300.48271562},
            {.95371294, -.00087671, .00278058, .30070451}},
        {"MAN_10t", false, .54, {500000.03771665, 4100000.02642698, 300.18058766},
            {.95345491, -.00704724, .02234861, .30062380}},
        {"MAN_10t", true, .42, {500000.00940168, 4100000.00659095, 300.43578978},
            {.95370567, -.00147020, .00465408, .30070212}},
    };
    for (const auto& baseline : baselines) {
        const auto truck = load_truck_geometry("viewer/assets/data/vehicle/MAN", baseline.vehicle);
        const auto seed = seed_for(flat, truck);
        const auto result = settle_vehicle(seed, flat, truck, baseline.lightweight, terrain);
        check(result.grounded && !result.settled, "unexpected resting placement status");
        check(std::abs(result.settling_time - baseline.time) < 1e-9, "settling step differs from Python");
        check(result.pose.support_clearance == 0, "resting pose retains drop clearance");
        check(result.pose.wheel_contacts.size() == truck.wheel_centers.size(),
              "wheel contact metadata lost");
        check(std::abs(result.pose.yaw - 35) < 0.05, "flat ground yaw changed");
        for (std::size_t i = 0; i < baseline.position.size(); ++i) {
            check(std::abs(result.pose.position[i] - baseline.position[i]) < 1e-4,
                  "settling position differs from Python by more than 0.1 mm");
        }
        for (std::size_t i = 0; i < baseline.orientation.size(); ++i) {
            check(std::abs(result.pose.orientation[i] - baseline.orientation[i]) < 1e-5,
                  "settling quaternion differs from Python");
        }

        rigid::State state;
        std::transform(result.pose.orientation.begin(), result.pose.orientation.end(),
                       state.orientation.begin(), [](double value) { return static_cast<float>(value); });
        TruckStepper stepper(result.pose.position);
        const auto step = stepper.step(flat, truck, state, {}, 0, 0, baseline.lightweight, terrain);
        double spring_force = 0;
        for (const auto& contact : step.physics.contacts) {
            spring_force += contact.spring[2];
        }
        check(spring_force > 100, "playback does not start in contact");
    }

    const auto truck = load_truck_geometry("viewer/assets/data/vehicle/MAN", "MAN_5t");
    const auto seed = seed_for(sparse, truck);
    const auto fallback = settle_vehicle(seed, sparse, truck, false, terrain);
    check(fallback.grounded && !fallback.settled && fallback.settling_time == 0,
          "uncovered footprint should fall back to geometric pose");
    check(fallback.pose.orientation == seed.orientation, "fallback should retain seed orientation");
    check(std::abs(fallback.pose.position[2] - (seed.position[2] - drop_clearance)) < 1e-8,
          "fallback should remove the drop clearance");
}
