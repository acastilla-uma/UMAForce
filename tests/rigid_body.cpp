#include "fusionforce/rigid_body.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace {

void near(float actual, float expected, float tolerance, const char* label)
{
    if (!(std::abs(actual - expected) <= tolerance)) {
        throw std::runtime_error(std::string(label) + ": " + std::to_string(actual)
                                 + " != " + std::to_string(expected));
    }
}

template<std::size_t N>
void near(const std::array<float, N>& actual, const std::array<float, N>& expected,
          float tolerance, const char* label)
{
    for (std::size_t i = 0; i < N; ++i) {
        near(actual[i], expected[i], tolerance, label);
    }
}

} // namespace

int main()
{
    using namespace fusionforce::rigid;

    const HeightGrid terrain(3, 3, 1, 1, {
        .2f, .23f, .34f,
        -.13f, 0, .21f,
        -.46f, -.23f, .08f,
    });
    near(terrain.sample(3, -2).height, .34f, 1e-6f, "heightmap edge clamp");
    near(terrain.sample(.5f, -.5f).height, .195f, 1e-6f, "heightmap bilinear sample");

    // Reference: CPU float32 DPhysicsEngine on z=.17x-.23y+.04x²+.1xy,
    // with supplied wheel geometry and no joints.
    const std::vector<ContactPoint> contacts{
        {{-.6f, .4f, -.28f}, {1.1f, .3f, 0}, terrain.sample(-.6f, .4f, {2100, .55f, .2f})},
        {{.8f, .7f, -.4f}, {1.9f, -.2f, 0}, terrain.sample(.8f, .7f, {1100, .65f, .35f})},
        {{.6f, -.8f, .25f}, {0, 0, 0}, terrain.sample(.6f, -.8f, {3000, .85f, .4f})},
        {{-.9f, -.55f, .04f}, {.4f, .8f, 0}, terrain.sample(-.9f, -.55f, {4100, .7f, .1f})},
    };
    const State state{{.2f, .3f, .4f}, {.25f, -.09f, -.12f},
                      {.946126997f, .183121353f, -.203468174f, .172947943f},
                      {.04f, -.03f, .14f}};
    const Mat3 inertia{{{33, 4, 1}, {4, 27, 5}, {1, 5, 42}}};
    const Vec3 cog{.12f, .08f, .6f};
    const Vec3 drive{1, 0, 0};
    const auto actual = step(state, {}, 200, cog, inertia, drive, contacts);

    const std::array<float, 4> weights{1, 1, .983055592f, .998098850f};
    const std::array<Vec3, 4> springs{{{-11.652359f, 20.859163f, 71.928146f},
                                       {-38.963047f, 19.225189f, 128.167923f},
                                       {-6.080184f, 7.490082f, 44.059315f},
                                       {-3.048765f, 22.688480f, 70.901512f}}};
    const std::array<Vec3, 4> frictions{{{30.647305f, 9.304472f, 2.266566f},
                                         {81.469124f, -.793736f, 24.885675f},
                                         {-12.988198f, -1.846155f, -1.478524f},
                                         {7.125955f, 6.148779f, -1.661193f}}};
    for (std::size_t i = 0; i < contacts.size(); ++i) {
        near(actual.contacts.at(i).contact_weight, weights[i], 1e-6f, "soft contact");
        near(actual.contacts[i].spring, springs[i], 1e-4f, "spring force");
        near(actual.contacts[i].friction, frictions[i], 1e-4f, "friction force");
    }
    near(actual.torque, {100.642731f, -55.300842f, -82.533043f}, 2e-4f, "torque");
    near(actual.acceleration, {.232549f, .415381f, -8.114653f}, 2e-6f, "acceleration");
    near(actual.angular_acceleration, {3.372602f, -2.217948f, -1.781331f}, 1e-5f, "angular acceleration");
    near(actual.state.position, {.202523261f, .299141556f, .397988528f}, 1e-6f, "position");
    near(actual.state.velocity, {.252325505f, -.085846193f, -.201146513f}, 1e-6f, "velocity");
    near(actual.state.orientation, {.945900500f, .183549270f, -.203666851f, .173498705f}, 1e-6f, "orientation");
    near(actual.state.angular_velocity, {.073726013f, -.052179478f, .122186691f}, 1e-6f, "angular velocity");

    EngineConfig clipped;
    clipped.torque_limit = 15;
    near(step(state, clipped, 200, cog, inertia, drive, contacts).torque,
         {15, -15, -15}, 0, "torque limit");
    const auto airborne = step({}, {}, 200, cog, inertia, drive, {{{0, 0, 1}, {}, {}}});
    near(airborne.contacts.at(0).contact_weight, 0, 0, "no contact");
    near(airborne.acceleration, {0, 0, -9.81f}, 1e-6f, "gravity only");

    const float half_sqrt = std::sqrt(.5f);
    const Mat3 rotation = rotation_matrix({half_sqrt, 0, 0, half_sqrt});
    near(transform(rotation, drive), {0, 1, 0}, 1e-6f, "heading");
    const Mat3 rotated = rotate_inertia(inertia, rotation);
    near(rotated[0], {27, -4, -5}, 1e-5f, "rotated inertia, x");
    near(rotated[1], {-4, 33, 1}, 1e-5f, "rotated inertia, y");
    near(rotated[2], {-5, 1, 42}, 1e-5f, "rotated inertia, z");
}
