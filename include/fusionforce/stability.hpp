#pragma once

#include <array>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace fusionforce {

using Vector3 = std::array<double, 3>;
using Quaternion = std::array<double, 4>;
using Matrix3 = std::array<Vector3, 3>;

struct StabilityDiagnostic {
    std::optional<std::variant<double, std::array<double, 2>>> value;
    std::map<std::string, std::optional<double>> params;
    std::optional<std::string> reason;
};

using StabilityDiagnostics = std::map<std::string, StabilityDiagnostic>;

// Collect one diagnostic sample per physics step, before playback subsampling.
class StabilityMonitor {
public:
    StabilityMonitor(double mass, const Matrix3& inertia_body, const Vector3& cog_body,
                     const std::vector<Vector3>& wheel_centers, double tire_radius,
                     std::size_t wheel_sample_count, double horizon = .5, double gravity = 9.81);

    // Orientation is a scalar-first quaternion. All vectors use SI units.
    StabilityDiagnostics update(double time, const std::vector<Vector3>& points,
                                const std::vector<Vector3>& forces, const Vector3& cog,
                                const Quaternion& orientation, const Vector3& velocity,
                                const Vector3& acceleration, const Vector3& angular_velocity);

private:
    StabilityDiagnostic ltr(const std::vector<Vector3>& forces, const char* no_support) const;
    StabilityDiagnostic pltr(double time, double ay, double phi, bool singular, const char* no_support);
    StabilityDiagnostic si(double phi, double omega_x, bool singular, const char* no_support) const;

    struct Sample {
        double time;
        double ay;
        double phi;
    };

    double mass_;
    double ixx_;
    double track_;
    double height_;
    double horizon_;
    double gravity_;
    std::vector<bool> left_;
    std::optional<Sample> previous_;
};

}  // namespace fusionforce
