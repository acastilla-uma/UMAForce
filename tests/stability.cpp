#include "fusionforce/stability.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

using namespace fusionforce;
const std::vector<Vector3> centers{{1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0}};
const std::vector<Vector3> loads(4, Vector3{0, 0, 10});
constexpr Matrix3 inertia{{{2, 0, 0}, {0, 3, 0}, {0, 0, 4}}};
const double pi = std::acos(-1.);

void check(bool condition, const char* description) {
    if (!condition) {
        throw std::runtime_error(description);
    }
}

void near(double actual, double expected, const char* description) {
    check(std::abs(actual-expected) <= 1e-12, description);
}

double scalar(const StabilityDiagnostic& diagnostic) {
    return std::get<double>(diagnostic.value.value());
}

Quaternion roll(double angle) { return {std::cos(angle/2), std::sin(angle/2), 0, 0}; }

StabilityMonitor monitor(double mass = 10, std::size_t samples = 4) {
    return {mass, inertia, {0, 0, .5}, centers, .5, samples};
}

StabilityDiagnostics sample(StabilityMonitor& diagnostic, double time = 0,
                            const std::vector<Vector3>& points = centers,
                            const std::vector<Vector3>& forces = loads,
                            const Quaternion& orientation = {1, 0, 0, 0},
                            const Vector3& acceleration = {},
                            const Vector3& angular_velocity = {},
                            const Vector3& cog = {0, 0, 1}) {
    return diagnostic.update(time, points, forces, cog, orientation, {}, acceleration, angular_velocity);
}

void loads_and_thresholds() {
    auto diagnostic = monitor();
    const auto result = sample(diagnostic);
    near(scalar(result.at("ltr")), 0, "symmetric LTR");
    near(scalar(result.at("si")), 1, "nominal SI");
    const auto& energy = result.at("si").params;
    near(*energy.at("phi_critical_rad"), pi/4, "critical roll");
    near(*energy.at("omega_critical_rad_s"), std::sqrt(2*10*9.81*(std::sqrt(2.)-1)/22),
         "critical roll rate");
    check(result.at("pltr").reason->find("anterior") != std::string::npos, "causal initialization");
    auto points = centers;
    points.emplace_back();
    const auto asymmetric = sample(diagnostic, .1, points,
        {{0, 0, 10}, {0, 0, 30}, {0, 0, 10}, {0, 0, 30}, {0, 0, 1000}});
    near(scalar(asymmetric.at("ltr")), .5, "LTR excludes chassis contacts");
    near(*asymmetric.at("ltr").params.at("fz_total_N"), 80, "wheel load total");
    auto full = monitor(10, 8);
    points.clear();
    for (const auto& point : centers) {
        points.insert(points.end(), 2, point);
    }
    points.emplace_back();
    near(scalar(sample(full, 0, points, {{0, 0, 5}, {0, 0, 5}, {0, 0, 15}, {0, 0, 15},
                {0, 0, 5}, {0, 0, 5}, {0, 0, 15}, {0, 0, 15}, {0, 0, 500}}).at("ltr")),
         .5, "full samples keep wheel identity");
}

void derivatives() {
    auto diagnostic = monitor();
    sample(diagnostic);
    constexpr double phi = .02;
    const auto result = sample(diagnostic, .1, centers, loads, roll(phi),
                               {0, .3*std::cos(phi), .3*std::sin(phi)});
    const auto& pltr = result.at("pltr");
    near(*pltr.params.at("ay_dot_m_s3"), 3, "lateral derivative");
    near(*pltr.params.at("phi_dot_rad_s"), .2, "roll derivative");
    near(scalar(pltr), .3/9.81 + phi + (3 + 9.81*.2)/9.81*.5, "predictive transfer ratio");
    auto wrapped = monitor();
    sample(wrapped, 0, centers, loads, roll(179*pi/180));
    const auto next = sample(wrapped, .1, centers, loads, roll(-179*pi/180));
    near(*next.at("pltr").params.at("phi_dot_rad_s"), 2*pi/180/.1, "roll unwrap");
    const auto repeated = sample(wrapped, .1);
    check(repeated.at("pltr").reason->find("creciente") != std::string::npos, "repeated timestamp");
    const auto invalid = sample(wrapped, .2, centers, loads, roll(0),
                                {std::numeric_limits<double>::quiet_NaN(), 0, 0});
    for (const auto& item : invalid) {
        check(!item.second.value && item.second.reason, "invalid sample");
    }
    check(!sample(wrapped, .3).at("pltr").value, "invalid sample resets history");
}

void wrench_and_unavailable() {
    auto diagnostic = monitor();
    auto points = centers;
    for (auto& point : points) {
        point[2] = .4;
    }
    const std::vector<Vector3> forces(4, Vector3{2, 3, 10});
    const auto result = sample(diagnostic, 0, points, forces);
    const auto& line = result.at("zml");
    near(*line.params.at("z_ref_m"), -.6, "ZML plane");
    near(*line.params.at("dx_dz"), .2, "ZML slope x");
    near(*line.params.at("dy_dz"), .3, "ZML slope y");
    near(*line.params.at("mx_Nm"), 7.2, "moment x");
    near(*line.params.at("my_Nm"), -4.8, "moment y");
    const auto coordinate = std::get<std::array<double, 2>>(*line.value);
    near(coordinate[0], 0, "ZML coordinate x");
    near(coordinate[1], 0, "ZML coordinate y");
    points = centers;
    points[1][2] = points[3][2] = .5;
    const auto weighted = sample(diagnostic, .1, points, {{0, 0, 10}, {0, 0, 30}, {0, 0, 10}, {0, 0, 30}});
    near(*weighted.at("zml").params.at("z_ref_m"), -.625, "weighted ZML plane");
    check(!sample(diagnostic, .2, centers, {{0, 0, 10}, {0, 0, -10}, {0, 0, 10}, {0, 0, -10}})
        .at("zml").value, "zero vertical resultant");
    const auto airborne = sample(diagnostic, .3, centers, std::vector<Vector3>(4));
    for (const auto& item : airborne) {
        check(!item.second.value && item.second.reason, "no support");
    }
    const auto singular = sample(diagnostic, .4, centers, loads, {std::cos(pi/4), 0, std::sin(pi/4), 0});
    check(!singular.at("si").value && !singular.at("pltr").value, "pitch singularity");
    auto invalid = monitor(std::numeric_limits<double>::quiet_NaN());
    const auto energy = sample(invalid).at("si");
    check(!energy.value && !energy.params.at("mass_kg"), "non-finite metadata is unavailable");
    check(scalar(sample(diagnostic, .5, centers, loads, roll(.2), {}, {4, 0, 0}).at("si")) < 0,
          "negative energy margin is preserved");
}

}  // namespace

int main() {
    try {
        loads_and_thresholds();
        derivatives();
        wrench_and_unavailable();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
