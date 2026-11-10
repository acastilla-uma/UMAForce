#include "fusionforce/truck_geometry.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {

void check(bool condition)
{
    if (!condition) {
        throw std::runtime_error("truck geometry check failed");
    }
}

} // namespace

int main()
{
    const std::array<std::string, 3> models{"MAN_5t", "MAN_7t", "MAN_10t"};
    for (std::size_t i = 0; i < models.size(); ++i) {
        const auto truck = fusionforce::load_truck_geometry("viewer/assets/data/vehicle/MAN", models[i]);
        check(truck.vehicle_id == models[i]);
        check(truck.wheel_centers.size() == 4 + 2 * i);
        check(truck.steering_factors.size() == truck.wheel_centers.size());
        check(truck.steering_factors.front() == 1.0f && truck.steering_factors.back() == 0.0f);
        check(truck.ring.size() == 48 && !truck.body.empty());
        check(std::abs(truck.ring[0][2] - truck.radius) < 1e-6);
        check(truck.wheel_centers[0][1] == -truck.wheel_centers[1][1]);
        check(truck.corners[0][0] > truck.corners[2][0]);
        check(truck.corners[0][1] > truck.corners[1][1]);
        check(truck.corners[0][2] == truck.corners[3][2]);
        const fusionforce::rigid::State state{{1, 2, 3}};
        const auto full = fusionforce::transform_truck(truck, state, 5, 0, false);
        check(full.contacts.size() == truck.wheel_centers.size() * 48 + truck.body.size());
        check(full.contacts.front().thrust == fusionforce::TruckPoint{5, 0, 0});
        check(full.contacts.back().thrust == fusionforce::TruckPoint{0, 0, 0});
        check(full.center_of_gravity[0] == truck.center_of_gravity[0] + 1);
        const auto light = fusionforce::transform_truck(truck, state, 5, 0.3f, true);
        check(light.contacts.size() == 4);
        check(light.contacts.front().thrust[1] > 0);
        check(light.contacts.back().thrust[1] == 0);
        if (models[i] == "MAN_10t") {
            check(truck.mass == 10785 && truck.moments_of_inertia[2] == 44746);
        }
    }
}
