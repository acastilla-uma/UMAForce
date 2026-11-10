#include "fusionforce/settling.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace fusionforce {
namespace {

constexpr double dt = 0.01;
constexpr int last_step = 200;
constexpr double compression_window = 0.35;

struct ContactPose {
    PlacementPoint position;
    std::array<double, 4> orientation;
    double time;
};

RestingPlacement placed(const PlacementSeed& seed, const ContactPose& contact)
{
    RestingPlacement result{seed};
    auto& pose = result.pose;
    pose.position = contact.position;
    pose.orientation = contact.orientation;
    pose.support_clearance = 0;
    result.settling_time = contact.time;

    // Match Rotation.from_quat: keep the published quaternion, normalize the
    // copy used to derive display angles and transform the wheel contacts.
    auto q = contact.orientation;
    const double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (auto& component : q) {
        component /= norm;
    }
    const auto [w, x, y, z] = q;
    const double r00 = w*w + x*x - y*y - z*z;
    const double r01 = 2 * (x*y - w*z);
    const double r02 = 2 * (x*z + w*y);
    const double r10 = 2 * (x*y + w*z);
    const double r11 = w*w - x*x + y*y - z*z;
    const double r12 = 2 * (y*z - w*x);
    const double r20 = 2 * (x*z - w*y);
    const double r21 = 2 * (y*z + w*x);
    const double r22 = w*w - x*x - y*y + z*z;

    constexpr double degrees = 180.0 / 3.14159265358979323846;
    const double pitch_cosine = std::hypot(r00, r10);
    pose.pitch = std::atan2(-r20, pitch_cosine) * degrees;
    if (pitch_cosine > 1e-7) {
        pose.roll = std::atan2(r21, r22) * degrees;
        pose.yaw = std::atan2(r10, r00) * degrees;
    } else {
        pose.roll = std::atan2(-r12, r11) * degrees;
        pose.yaw = 0;
    }

    pose.wheel_contacts.clear();
    pose.wheel_contacts.reserve(pose.wheel_centers.size());
    for (const auto& wheel : pose.wheel_centers) {
        const double bottom = wheel[2] - pose.tire_radius;
        pose.wheel_contacts.push_back({
            pose.position[0] + r00*wheel[0] + r01*wheel[1] + r02*bottom,
            pose.position[1] + r10*wheel[0] + r11*wheel[1] + r12*bottom,
            pose.position[2] + r20*wheel[0] + r21*wheel[1] + r22*bottom,
        });
    }
    return result;
}

} // namespace

RestingPlacement settle_vehicle(const PlacementSeed& seed, const CloudStore& cloud,
                                const TruckGeometry& truck, bool lightweight,
                                const TruckTerrain& terrain)
{
    rigid::EngineConfig config;
    config.dt = static_cast<float>(dt);
    config.torque_limit = 1e7f;
    rigid::State state;
    std::transform(seed.orientation.begin(), seed.orientation.end(), state.orientation.begin(),
                   [](double value) { return static_cast<float>(value); });
    TruckStepper stepper(seed.position);
    std::optional<double> first_contact;
    std::optional<ContactPose> last_contact;
    for (int i = 0; i <= last_step; ++i) {
        try {
            auto result = stepper.step(cloud, truck, state, config, 0, 0, lightweight, terrain);
            float vertical_force = 0;
            for (const auto& contact : result.physics.contacts) {
                vertical_force += contact.spring[2] + contact.friction[2];
            }
            if (vertical_force > 0.05 * truck.mass * 9.81) {
                const double time = i * dt;
                if (!first_contact) {
                    first_contact = time;
                }
                last_contact = ContactPose{
                    {seed.position[0] + state.position[0], seed.position[1] + state.position[1],
                     seed.position[2] + state.position[2]},
                    {state.orientation[0], state.orientation[1], state.orientation[2], state.orientation[3]},
                    time,
                };
                if (result.physics.state.velocity[2] >= 0 || time - *first_contact >= compression_window) {
                    break;
                }
            }
            state = result.physics.state;
        } catch (const TruckCoverageError&) {
            break;
        } catch (const TruckInstabilityError&) {
            break;
        }
    }
    if (last_contact) {
        return placed(seed, *last_contact);
    }
    auto ground = seed.position;
    ground[2] -= drop_clearance;
    return placed(seed, {ground, seed.orientation, 0});
}

} // namespace fusionforce
