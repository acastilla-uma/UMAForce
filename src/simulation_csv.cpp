#include "fusionforce/simulation_csv.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <ostream>
#include <stdexcept>
#include <string_view>
#include <type_traits>

namespace fusionforce {
namespace {

constexpr std::string_view header =
    "time_s,record,contact_id,x_m,y_m,z_m,qw,qx,qy,qz,vx_m_s,vy_m_s,vz_m_s,wx_rad_s,"
    "wy_rad_s,wz_rad_s,ax_m_s2,ay_m_s2,az_m_s2,alphax_rad_s2,alphay_rad_s2,alphaz_rad_s2,"
    "fx_N,fy_N,fz_N,mx_Nm,my_Nm,mz_Nm,normal_x_N,normal_y_N,normal_z_N,friction_x_N,"
    "friction_y_N,friction_z_N,contact_weight,terrain_class,friction_mu,stiffness_N_m,"
    "target_x_m,target_y_m,target_z_m,target_speed_m_s,target_acceleration_m_s2,"
    "tracking_error_m,speed_error_m_s,tangential_acceleration_m_s2,acceleration_error_m_s2";

constexpr std::array<std::string_view, 4> corner_ids{
    "front_left", "front_right", "rear_left", "rear_right"};

void check_output(const std::ostream& output)
{
    if (!output) {
        throw std::runtime_error("No se pudo escribir el CSV de simulación");
    }
}

template <class Number>
void append_number(std::string& row, Number value)
{
    char buffer[32];
    const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value,
                                      std::chars_format::scientific);
    if (result.ec != std::errc{}) {
        throw std::runtime_error("No se pudo escribir un valor numérico del CSV");
    }
    const std::string_view scientific(buffer, result.ptr - buffer);
    const auto exponent_at = scientific.find('e');
    if (exponent_at == std::string_view::npos) {
        row.append(scientific); // nan and inf
        return;
    }
    int exponent = 0;
    const char* exponent_start = buffer + exponent_at + 1;
    if (*exponent_start == '+') {
        ++exponent_start;
    }
    std::from_chars(exponent_start, result.ptr, exponent);
    bool fixed = exponent >= -4 && exponent < 16;
    if constexpr (std::is_same_v<Number, float>) {
        // NumPy scalars choose notation using the unrounded magnitude.
        const double absolute = std::abs(static_cast<double>(value));
        fixed = absolute == 0 || (absolute >= 1e-4 && absolute < 1e16);
    }
    if (!fixed) {
        row.append(scientific);
        return;
    }

    auto mantissa = scientific.substr(0, exponent_at);
    if (mantissa.front() == '-') {
        row.push_back('-');
        mantissa.remove_prefix(1);
    }
    char digits[24];
    const auto digits_end = std::remove_copy(mantissa.begin(), mantissa.end(), digits, '.');
    const auto count = static_cast<int>(digits_end - digits);
    const int decimal = exponent + 1;
    if (decimal <= 0) {
        row.append("0.");
        row.append(-decimal, '0');
        row.append(digits, count);
    } else if (decimal < count) {
        row.append(digits, decimal);
        row.push_back('.');
        row.append(digits + decimal, count - decimal);
    } else {
        row.append(digits, count);
        row.append(decimal - count, '0');
        row.append(".0");
    }
}

class CsvRow {
public:
    explicit CsvRow(std::string& row) : row_(row)
    {
        row_.clear();
    }

    void field(std::string_view value)
    {
        separator();
        if (value.find_first_of(",\"\r\n") == std::string_view::npos) {
            row_.append(value);
            return;
        }
        row_.push_back('"');
        for (char character : value) {
            if (character == '"') {
                row_.push_back('"');
            }
            row_.push_back(character);
        }
        row_.push_back('"');
    }

    void empty(int count)
    {
        for (int i = 0; i < count; ++i) {
            separator();
        }
    }

    template <class Number>
    void number(Number value)
    {
        separator();
        append_number(row_, value);
    }

    template <class Number, std::size_t Size>
    void numbers(const std::array<Number, Size>& values)
    {
        for (const auto value : values) {
            number(value);
        }
    }

    // Python converts Tensor.tolist() elements to double before formatting.
    template <std::size_t Size>
    void promoted(const std::array<float, Size>& values)
    {
        for (const auto value : values) {
            number(static_cast<double>(value));
        }
    }

    void contact_id(std::size_t index, bool lightweight)
    {
        if (lightweight) {
            field(corner_ids[index]);
            return;
        }
        char digits[32];
        const auto result = std::to_chars(digits, digits + sizeof(digits), index);
        const auto count = result.ptr - digits;
        separator();
        row_.push_back('p');
        if (count < 4) {
            row_.append(4 - count, '0');
        }
        row_.append(digits, count);
    }

    void send(std::ostream& output)
    {
        row_.append("\r\n");
        output.write(row_.data(), static_cast<std::streamsize>(row_.size()));
        check_output(output);
    }

private:
    void separator()
    {
        if (first_) {
            first_ = false;
        } else {
            row_.push_back(',');
        }
    }

    std::string& row_;
    bool first_ = true;
};

rigid::Vec3 contact_force(const rigid::ContactForces& contact)
{
    return {contact.spring[0] + contact.friction[0], contact.spring[1] + contact.friction[1],
            contact.spring[2] + contact.friction[2]};
}

void write_route(CsvRow& row, const SimulationFrame& frame, bool lightweight)
{
    const auto& command = frame.command;
    if (command.target) {
        const auto& target = *command.target;
        row.numbers(std::array{target.x, target.y, target.z, target.speed, target.acceleration});
    } else {
        row.empty(5);
    }
    row.number(command.tracking_error);
    row.number(command.speed_error);
    row.number(frame.tangential_acceleration);
    row.number(frame.acceleration_error);
    if (lightweight) {
        row.field("four_corners");
    }
}

} // namespace

SimulationCsvWriter::SimulationCsvWriter(std::ostream& output, bool lightweight)
    : output_(output), lightweight_(lightweight)
{
    row_.reserve(1024);
    output_ << header;
    if (lightweight_) {
        output_ << ",contact_mode";
    }
    output_ << "\r\n";
    check_output(output_);
}

void SimulationCsvWriter::write(const SimulationFrame& frame)
{
    const auto& physics = frame.step.physics;
    const auto count = physics.contacts.size();
    if (frame.step.global_points.size() != count || frame.step.contact_materials.size() != count ||
        frame.step.moments.size() != count || (lightweight_ && count != corner_ids.size())) {
        throw std::invalid_argument("Contactos de simulación inválidos para CSV");
    }

    rigid::Vec3 total{};
    for (const auto& contact : physics.contacts) {
        const auto force = contact_force(contact);
        for (std::size_t axis = 0; axis < total.size(); ++axis) {
            total[axis] += force[axis];
        }
    }

    CsvRow body(row_);
    body.number(frame.time);
    body.field("body");
    body.empty(1);
    body.numbers(frame.global_position);
    body.promoted(frame.state.orientation);
    body.promoted(frame.state.velocity);
    body.promoted(frame.state.angular_velocity);
    body.promoted(physics.acceleration);
    body.promoted(physics.angular_acceleration);
    body.numbers(total);
    body.promoted(physics.torque);
    body.empty(10); // Normal and friction vectors, contact weight and terrain properties.
    write_route(body, frame, lightweight_);
    body.send(output_);

    for (std::size_t i = 0; i < count; ++i) {
        const auto& point = frame.step.global_points[i];
        const auto& forces = physics.contacts[i];
        const auto& material = frame.step.contact_materials[i];
        const auto force = contact_force(forces);
        CsvRow row(row_);
        row.number(frame.time);
        row.field("contact");
        row.contact_id(i, lightweight_);
        row.numbers(point);
        row.empty(16); // Orientation, velocities and accelerations.
        row.numbers(force);
        row.numbers(frame.step.moments[i]);
        row.numbers(forces.spring);
        row.numbers(forces.friction);
        row.number(static_cast<double>(forces.contact_weight));
        row.field(material.terrain_class);
        row.number(static_cast<double>(material.material.friction_lon));
        row.number(static_cast<double>(material.material.stiffness));
        write_route(row, frame, lightweight_);
        row.send(output_);
    }
}

} // namespace fusionforce
