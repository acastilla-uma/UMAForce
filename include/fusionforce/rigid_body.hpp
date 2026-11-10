#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace fusionforce::rigid {

using Vec3 = std::array<float, 3>;
using Mat3 = std::array<Vec3, 3>;
using Quaternion = std::array<float, 4>; // w, x, y, z

struct State {
    Vec3 position{};
    Vec3 velocity{};
    Quaternion orientation{1, 0, 0, 0};
    Vec3 angular_velocity{};
};

struct EngineConfig {
    float dt = .01f;
    float gravity = 9.81f;
    float torque_limit = 200.f;
    float damping_alpha = 1.f;
    float soft_contact_sigma = .01f;
};

struct Material {
    float stiffness = 40000.f;
    float friction_lon = .5f;
    float friction_lat = .2f;
};

struct TerrainSample {
    float height = 0;
    Vec3 normal{0, 0, 1};
    Material material;
};

class HeightGrid {
public:
    // Row-major heights in increasing Y and X, with a centered query domain.
    HeightGrid(std::size_t width, std::size_t height, float grid_res,
               float max_coord, std::vector<float> heights);
    TerrainSample sample(float x, float y, Material material = {}) const;

private:
    std::size_t width_;
    std::size_t height_;
    float max_coord_;
    std::vector<float> heights_;
    std::vector<float> gradient_x_;
    std::vector<float> gradient_y_;
};

struct ContactPoint {
    Vec3 position;
    Vec3 thrust;
    TerrainSample terrain;
};

struct ContactForces {
    Vec3 spring;
    Vec3 friction;
    float contact_weight;
};

struct StepResult {
    State state;
    Vec3 acceleration;
    Vec3 angular_acceleration;
    Vec3 torque;
    std::vector<ContactForces> contacts;
};

Mat3 rotation_matrix(const Quaternion& orientation);
Vec3 transform(const Mat3& matrix, const Vec3& vector);
Mat3 rotate_inertia(const Mat3& local, const Mat3& rotation);
StepResult step(const State& state, const EngineConfig& config, float mass,
                const Vec3& world_cog, const Mat3& world_inertia,
                const Vec3& local_driving_direction, const std::vector<ContactPoint>& contacts);

} // namespace fusionforce::rigid
