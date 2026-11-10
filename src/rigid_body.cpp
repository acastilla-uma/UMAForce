#include "fusionforce/rigid_body.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace fusionforce::rigid {
namespace {

Vec3 add(Vec3 a, Vec3 b)
{
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 subtract(Vec3 a, Vec3 b)
{
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 scale(Vec3 v, float s)
{
    return {v[0] * s, v[1] * s, v[2] * s};
}

float dot(Vec3 a, Vec3 b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 cross(Vec3 a, Vec3 b)
{
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

Vec3 normalized(Vec3 v)
{
    const float norm = std::max(std::sqrt(dot(v, v)), 1e-8f);
    return {v[0] / norm, v[1] / norm, v[2] / norm};
}

Vec3 rotate_direction(const Quaternion& q, Vec3 vector)
{
    const Vec3 xyz{q[1], q[2], q[3]};
    const Vec3 uv = cross(xyz, vector);
    return add(vector, scale(add(scale(uv, q[0]), cross(xyz, uv)), 2));
}

Quaternion integrate(const Quaternion& q, Vec3 omega, float dt)
{
    const Vec3 half = scale(omega, .5f * dt);
    const float theta = std::sqrt(dot(half, half));
    const float sinc = theta > 1e-8f ? std::sin(theta) / theta : 1.f;
    const Vec3 delta = scale(half, sinc);
    const float scalar = std::cos(theta);
    const Vec3 xyz{q[1], q[2], q[3]};
    const Vec3 product = add(add(scale(xyz, scalar), scale(delta, q[0])), cross(delta, xyz));
    const float w = scalar * q[0] - dot(delta, xyz);
    const float norm = std::max(std::sqrt(w * w + dot(product, product)), 1e-8f);
    return {w / norm, product[0] / norm, product[1] / norm, product[2] / norm};
}

Vec3 solve(Mat3 a, Vec3 b)
{
    for (std::size_t column = 0; column < a.size(); ++column) {
        std::size_t pivot = column;
        for (std::size_t row = column + 1; row < a.size(); ++row) {
            if (std::abs(a[row][column]) > std::abs(a[pivot][column])) {
                pivot = row;
            }
        }
        std::swap(a[pivot], a[column]);
        std::swap(b[pivot], b[column]);
        if (a[column][column] == 0) {
            const float invalid = std::numeric_limits<float>::quiet_NaN();
            return {invalid, invalid, invalid};
        }
        for (std::size_t row = column + 1; row < a.size(); ++row) {
            const float multiplier = a[row][column] / a[column][column];
            for (std::size_t j = column + 1; j < a.size(); ++j) {
                a[row][j] -= multiplier * a[column][j];
            }
            b[row] -= multiplier * b[column];
        }
    }
    Vec3 x{};
    for (std::size_t i = a.size(); i-- > 0;) {
        float value = b[i];
        for (std::size_t j = i + 1; j < a.size(); ++j) {
            value -= a[i][j] * x[j];
        }
        x[i] = value / a[i][i];
    }
    return x;
}

float gradient(const std::vector<float>& values, std::size_t index,
               std::size_t axis_index, std::size_t axis_size, std::size_t stride, float spacing)
{
    if (axis_index == 0) {
        return (-1.5f * values[index] + 2 * values[index + stride]
                - .5f * values[index + 2 * stride]) / spacing;
    }
    if (axis_index + 1 == axis_size) {
        return (.5f * values[index - 2 * stride] - 2 * values[index - stride]
                + 1.5f * values[index]) / spacing;
    }
    return (values[index + stride] - values[index - stride]) / (2 * spacing);
}

} // namespace

HeightGrid::HeightGrid(std::size_t width, std::size_t height, float grid_res,
                       float max_coord, std::vector<float> heights)
    : width_(width), height_(height), max_coord_(max_coord), heights_(std::move(heights))
{
    if (width < 3 || height < 3 || width > heights_.size() / height
        || width * height != heights_.size() || !(grid_res > 0) || !(max_coord > 0)) {
        throw std::invalid_argument("Invalid height grid dimensions or scale");
    }
    gradient_x_.resize(heights_.size());
    gradient_y_.resize(heights_.size());
    for (std::size_t i = 0; i < heights_.size(); ++i) {
        gradient_x_[i] = gradient(heights_, i, i % width, width, 1, grid_res);
        gradient_y_[i] = gradient(heights_, i, i / width, height, width, grid_res);
    }
}

TerrainSample HeightGrid::sample(float x, float y, Material material) const
{
    if (!std::isfinite(x) || !std::isfinite(y)) {
        throw std::invalid_argument("Non-finite terrain query");
    }
    const float column = (std::clamp(x / max_coord_, -1.f, 1.f) + 1) * .5f * (width_ - 1);
    const float row = (std::clamp(y / max_coord_, -1.f, 1.f) + 1) * .5f * (height_ - 1);
    const auto left = static_cast<std::size_t>(column);
    const auto top = static_cast<std::size_t>(row);
    const auto right = std::min(left + 1, width_ - 1);
    const auto bottom = std::min(top + 1, height_ - 1);
    const float u = column - left;
    const float v = row - top;
    const auto interpolate = [&](const std::vector<float>& values) {
        return (1 - u) * (1 - v) * values[top * width_ + left]
             + u * (1 - v) * values[top * width_ + right]
             + (1 - u) * v * values[bottom * width_ + left]
             + u * v * values[bottom * width_ + right];
    };
    return {interpolate(heights_), normalized({-interpolate(gradient_x_), -interpolate(gradient_y_), 1}), material};
}

Mat3 rotation_matrix(const Quaternion& q)
{
    const auto [w, x, y, z] = q;
    const float ww = w * w, xx = x * x, yy = y * y, zz = z * z;
    return {{{ww + xx - yy - zz, 2 * (x * y - w * z), 2 * (x * z + w * y)},
             {2 * (x * y + w * z), ww - xx + yy - zz, 2 * (y * z - w * x)},
             {2 * (x * z - w * y), 2 * (y * z + w * x), ww - xx - yy + zz}}};
}

Vec3 transform(const Mat3& matrix, const Vec3& vector)
{
    return {dot(matrix[0], vector), dot(matrix[1], vector), dot(matrix[2], vector)};
}

Mat3 rotate_inertia(const Mat3& local, const Mat3& rotation)
{
    const Mat3 columns{{{local[0][0], local[1][0], local[2][0]},
                        {local[0][1], local[1][1], local[2][1]},
                        {local[0][2], local[1][2], local[2][2]}}};
    Mat3 result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
        const Vec3 row = transform(columns, rotation[i]);
        for (std::size_t j = 0; j < result.size(); ++j) {
            result[i][j] = dot(row, rotation[j]);
        }
    }
    return result;
}

StepResult step(const State& state, const EngineConfig& config, float mass,
                const Vec3& world_cog, const Mat3& world_inertia,
                const Vec3& local_driving_direction, const std::vector<ContactPoint>& contacts)
{
    StepResult result{};
    result.contacts.resize(contacts.size());
    float contact_count = 0;
    for (std::size_t i = 0; i < contacts.size(); ++i) {
        const auto& point = contacts[i];
        const float depth = (point.position[2] - point.terrain.height) * point.terrain.normal[2];
        const float weight = .5f * (1 + std::tanh(-depth / config.soft_contact_sigma * std::sqrt(3.f)));
        result.contacts[i].contact_weight = weight;
        contact_count += weight;
    }
    contact_count = std::max(contact_count, 1.f);
    const Vec3 driving = rotate_direction(state.orientation, local_driving_direction);
    Vec3 force{};
    for (std::size_t i = 0; i < contacts.size(); ++i) {
        const auto& point = contacts[i];
        const auto& material = point.terrain.material;
        const Vec3 normal = point.terrain.normal;
        auto& forces = result.contacts[i];
        const Vec3 lever = subtract(point.position, world_cog);
        const Vec3 velocity = add(state.velocity, cross(state.angular_velocity, lever));
        const float depth = (point.position[2] - point.terrain.height) * normal[2] * forces.contact_weight;
        const float damping = config.damping_alpha * 2 * std::sqrt(mass * material.stiffness / contact_count);
        const float spring = -(material.stiffness * depth + damping * dot(velocity, normal));
        forces.spring = scale(scale(normal, spring * forces.contact_weight), 1.f / contact_count);

        const Vec3 forward = normalized(subtract(driving, scale(normal, dot(driving, normal))));
        const Vec3 lateral = normalized(cross(forward, normal));
        const Vec3 relative = subtract(point.thrust, velocity);
        const Vec3 tangent = subtract(relative, scale(normal, dot(relative, normal)));
        const Vec3 slip{std::tanh(tangent[0]), std::tanh(tangent[1]), std::tanh(tangent[2])};
        const float magnitude = std::sqrt(dot(forces.spring, forces.spring));
        const Vec3 longitudinal = scale(scale(forward, dot(slip, forward)), material.friction_lon * magnitude);
        const Vec3 transverse = scale(scale(lateral, dot(slip, lateral)), material.friction_lat * magnitude);
        forces.friction = add(transverse, longitudinal);
        const Vec3 total = add(forces.spring, forces.friction);
        result.torque = add(result.torque, cross(lever, total));
        force = add(force, total);
    }
    for (float& torque : result.torque) {
        torque = std::clamp(torque, -config.torque_limit, config.torque_limit);
    }
    force[2] += -mass * config.gravity;
    result.acceleration = {force[0] / mass, force[1] / mass, force[2] / mass};
    result.angular_acceleration = solve(world_inertia, result.torque);
    result.state.velocity = add(state.velocity, scale(result.acceleration, config.dt));
    result.state.position = add(state.position, scale(result.state.velocity, config.dt));
    result.state.angular_velocity = add(state.angular_velocity, scale(result.angular_acceleration, config.dt));
    result.state.orientation = integrate(state.orientation, result.state.angular_velocity, config.dt);
    return result;
}

} // namespace fusionforce::rigid
