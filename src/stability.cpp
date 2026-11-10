#include "fusionforce/stability.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <type_traits>
#include <utility>

namespace fusionforce {
namespace {

constexpr double eps = 1e-8;
constexpr auto singular_reason = "Balanceo Euler no definido cerca de cabeceo ±90°";
constexpr auto invalid_reason = "Datos físicos ausentes o no finitos";
using Params = decltype(StabilityDiagnostic::params);
using Value = decltype(StabilityDiagnostic::value);

bool finite(double value) { return std::isfinite(value); }

template <std::size_t N>
bool all_finite(const std::array<double, N>& values) {
    return std::all_of(values.begin(), values.end(), finite);
}

bool positive(double value) { return finite(value) && value > 0; }

double dot(const Vector3& a, const Vector3& b) {
    return std::inner_product(a.begin(), a.end(), b.begin(), 0.);
}

StabilityDiagnostic entry(Value value, Params params = {}, const char* reason = nullptr) {
    for (auto& item : params) {
        if (item.second && !finite(*item.second)) {
            item.second.reset();
        }
    }
    if (reason) {
        return {std::nullopt, std::move(params), reason};
    }
    if (value && std::visit([](const auto& result) {
            using T = std::decay_t<decltype(result)>;
            if constexpr (std::is_same_v<T, double>) {
                return finite(result);
            } else {
                return all_finite(result);
            }
        }, *value)) {
        return {value, std::move(params), std::nullopt};
    }
    return {std::nullopt, std::move(params), "Resultado no finito"};
}

StabilityDiagnostic zml(const std::vector<Vector3>& points, const std::vector<Vector3>& forces,
                        const Vector3& cog, double yaw, double weight, const char* no_support) {
    const double c = std::cos(yaw), s = std::sin(yaw);
    Vector3 force{}, moment{};
    double weighted_z = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto& point = points[i];
        const auto& load = forces[i];
        const double dx = point[0] - cog[0], dy = point[1] - cog[1];
        const Vector3 p{dx*c + dy*s, -dx*s + dy*c, point[2] - cog[2]};
        const Vector3 f{load[0]*c + load[1]*s, -load[0]*s + load[1]*c, load[2]};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            force[axis] += f[axis];
            const auto next = (axis + 1) % 3;
            const auto last = (axis + 2) % 3;
            moment[axis] += p[next]*f[last] - p[last]*f[next];
        }
        weighted_z += std::max(f[2], 0.)*p[2];
    }
    const auto [fx, fy, fz] = force;
    const auto [mx, my, mz] = moment;
    Params params{{"fx_N", fx}, {"fy_N", fy}, {"fz_N", fz}, {"mx_Nm", mx},
                  {"my_Nm", my}, {"mz_Nm", mz}, {"z_ref_m", {}}, {"x_ref_m", {}},
                  {"y_ref_m", {}}, {"dx_dz", {}}, {"dy_dz", {}}, {"torsion_residual_Nm", {}},
                  {"cog_x_m", cog[0]}, {"cog_y_m", cog[1]}, {"cog_z_m", cog[2]}, {"yaw_rad", yaw}};
    if (no_support) {
        return entry({}, std::move(params), no_support);
    }
    if (std::abs(fz) <= eps) {
        return entry({}, std::move(params), "Resultante vertical nula: línea no definida");
    }
    const double z_ref = weighted_z/weight;
    const double x_ref = (-my + z_ref*fx)/fz;
    const double y_ref = (mx + z_ref*fy)/fz;
    params["z_ref_m"] = z_ref;
    params["x_ref_m"] = x_ref;
    params["y_ref_m"] = y_ref;
    params["dx_dz"] = fx/fz;
    params["dy_dz"] = fy/fz;
    params["torsion_residual_Nm"] = dot(moment, force)/fz;
    return entry(std::array<double, 2>{x_ref, y_ref}, std::move(params));
}

}  // namespace

StabilityMonitor::StabilityMonitor(double mass, const Matrix3& inertia_body, const Vector3& cog_body,
                                   const std::vector<Vector3>& centers, double tire_radius,
                                   std::size_t wheel_sample_count, double horizon, double gravity)
    : mass_(mass), ixx_(inertia_body[0][0]), track_(std::numeric_limits<double>::quiet_NaN()),
      height_(track_), horizon_(horizon), gravity_(gravity) {
    if (centers.empty() || !all_finite(cog_body) ||
        !std::all_of(centers.begin(), centers.end(), all_finite<3>)) {
        return;
    }
    const auto by_y = [](const Vector3& a, const Vector3& b) { return a[1] < b[1]; };
    const auto by_z = [](const Vector3& a, const Vector3& b) { return a[2] < b[2]; };
    const auto range = std::minmax_element(centers.begin(), centers.end(), by_y);
    track_ = (*range.second)[1] - (*range.first)[1];
    height_ = cog_body[2] - (*std::min_element(centers.begin(), centers.end(), by_z))[2] + tire_radius;
    if (wheel_sample_count == 4) {
        left_ = {true, false, true, false};
    } else if (wheel_sample_count > 0 && wheel_sample_count % centers.size() == 0) {
        const auto per_wheel = wheel_sample_count/centers.size();
        for (const auto& center : centers) {
            left_.insert(left_.end(), per_wheel, center[1] > 0);
        }
    }
}

StabilityDiagnostics StabilityMonitor::update(double time, const std::vector<Vector3>& points,
                                              const std::vector<Vector3>& forces, const Vector3& cog,
                                              const Quaternion& orientation, const Vector3& velocity,
                                              const Vector3& acceleration, const Vector3& angular_velocity) {
    const double norm = std::sqrt(std::inner_product(orientation.begin(), orientation.end(),
                                                    orientation.begin(), 0.));
    if (!finite(time) || !(norm > eps) || !all_finite(orientation) || !all_finite(cog) ||
        !all_finite(velocity) || !all_finite(acceleration) || !all_finite(angular_velocity) ||
        points.size() != forces.size() ||
        !std::all_of(points.begin(), points.end(), all_finite<3>) ||
        !std::all_of(forces.begin(), forces.end(), all_finite<3>)) {
        previous_.reset();
        const auto invalid = entry({}, {}, invalid_reason);
        return {{"ltr", invalid}, {"pltr", invalid}, {"zml", invalid}, {"si", invalid}};
    }
    const auto [qw, qx, qy, qz] = orientation;
    const double w = qw/norm, x = qx/norm, y = qy/norm, z = qz/norm;
    const Vector3 col0{1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)};
    const Vector3 col1{2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x)};
    const double phi = std::atan2(col1[2], 1-2*(x*x+y*y));
    const double yaw = std::atan2(col0[1], col0[0]);
    const bool singular = std::abs(col0[2]) >= 1-eps;
    const double weight = std::accumulate(forces.begin(), forces.end(), 0.,
        [](double sum, const Vector3& force) { return sum + std::max(force[2], 0.); });
    const char* no_support = weight > eps ? nullptr : "Sin apoyo vertical: índice no disponible";
    return {{"ltr", ltr(forces, no_support)},
            {"pltr", pltr(time, dot(col1, acceleration), phi, singular, no_support)},
            {"si", si(phi, dot(col0, angular_velocity), singular, no_support)},
            {"zml", zml(points, forces, cog, yaw, weight, no_support)}};
}

StabilityDiagnostic StabilityMonitor::ltr(const std::vector<Vector3>& forces, const char* no_support) const {
    const auto left_count = static_cast<std::size_t>(std::count(left_.begin(), left_.end(), true));
    if (left_.size() > forces.size() || left_count == 0 || left_count == left_.size()) {
        return entry({}, {}, "No se puede identificar el lado de las ruedas");
    }
    double left = 0, right = 0;
    for (std::size_t i = 0; i < left_.size(); ++i) {
        (left_[i] ? left : right) += forces[i][2];
    }
    const double total = left + right;
    Params params{{"fz_left_N", left}, {"fz_right_N", right}, {"fz_total_N", total}};
    const char* reason = no_support;
    if (!reason && total <= eps) {
        reason = "Ruedas sin carga vertical";
    }
    return entry(reason ? Value{} : Value{(right-left)/total}, std::move(params), reason);
}

StabilityDiagnostic StabilityMonitor::pltr(double time, double ay, double phi, bool singular,
                                           const char* no_support) {
    Params params{{"h_m", height_}, {"track_m", track_}, {"gravity_m_s2", gravity_},
                  {"ay_body_m_s2", ay}, {"phi_rad", phi}, {"ay_dot_m_s3", {}},
                  {"phi_dot_rad_s", {}}, {"horizon_s", horizon_}, {"derivative_dt_s", {}},
                  {"base_ltr_kinematic", {}}, {"prediction_term", {}}};
    const auto previous = previous_;
    if (singular) {
        previous_.reset();
        return entry({}, std::move(params), singular_reason);
    }
    previous_ = Sample{time, ay, phi};
    if (!positive(height_) || !positive(track_) || !positive(gravity_) || !positive(horizon_)) {
        return entry({}, std::move(params), "Geometría, gravedad u horizonte no válidos");
    }
    if (!previous) {
        return entry({}, std::move(params), "Se necesita una muestra anterior para la derivada causal");
    }
    const double elapsed = time - previous->time;
    if (elapsed <= eps) {
        return entry({}, std::move(params), "Tiempo no creciente: derivada no disponible");
    }
    const double ay_dot = (ay - previous->ay)/elapsed;
    const double phi_delta = phi - previous->phi;
    const double phi_dot = std::atan2(std::sin(phi_delta), std::cos(phi_delta))/elapsed;
    const double base = 2*height_/track_*(ay/gravity_ + phi);
    const double prediction = 2*height_/(track_*gravity_)*(ay_dot + gravity_*phi_dot)*horizon_;
    params["ay_dot_m_s3"] = ay_dot;
    params["phi_dot_rad_s"] = phi_dot;
    params["derivative_dt_s"] = elapsed;
    params["base_ltr_kinematic"] = base;
    params["prediction_term"] = prediction;
    return entry(base + prediction, std::move(params), no_support);
}

StabilityDiagnostic StabilityMonitor::si(double phi, double omega_x, bool singular,
                                         const char* no_support) const {
    const double half_track = track_/2;
    constexpr double k1 = 1.15, k2 = 2.05;
    Params params{{"mass_kg", mass_}, {"half_track_m", half_track}, {"h_m", height_},
                  {"distance_m", {}}, {"inertia_cg_kg_m2", ixx_}, {"inertia_pivot_kg_m2", {}},
                  {"gravity_m_s2", gravity_}, {"phi_rad", phi}, {"omega_body_x_rad_s", omega_x},
                  {"phi_critical_rad", {}}, {"omega_critical_rad_s", {}}, {"k1", k1}, {"k2", k2},
                  {"roll_term", {}}, {"omega_term", {}}};
    if (!positive(mass_) || !positive(track_) || !positive(height_) ||
        !positive(ixx_) || !positive(gravity_)) {
        return entry({}, std::move(params), "Masa, geometría o inercia no válidas");
    }
    const double distance = std::hypot(half_track, height_);
    const double pivot_inertia = ixx_ + mass_*(distance*distance);
    const double phi_critical = std::atan2(half_track, height_);
    const double omega_critical = std::sqrt(2*mass_*gravity_*(distance-height_)/pivot_inertia);
    if (omega_critical <= eps) {
        return entry({}, std::move(params), "Umbral energético degenerado");
    }
    const double roll_term = k1*std::abs(phi)/phi_critical;
    const double ratio = omega_x/omega_critical;
    const double omega_term = k2*(ratio*ratio);
    params["distance_m"] = distance;
    params["inertia_pivot_kg_m2"] = pivot_inertia;
    params["phi_critical_rad"] = phi_critical;
    params["omega_critical_rad_s"] = omega_critical;
    params["roll_term"] = roll_term;
    params["omega_term"] = omega_term;
    return entry(1-roll_term-omega_term, std::move(params), singular ? singular_reason : no_support);
}

}  // namespace fusionforce
