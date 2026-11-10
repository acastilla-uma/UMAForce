#include "fusionforce/projects.hpp"
#include "fusionforce/sha256.hpp"

#include <zlib.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace fusionforce {
namespace {

using nlohmann::json;

constexpr int archive_version = 2;
constexpr std::string_view archive_format = "fusionforce-project";
constexpr std::uint32_t local_signature = 0x04034b50;
constexpr std::uint32_t directory_signature = 0x02014b50;
constexpr std::uint32_t end_signature = 0x06054b50;
constexpr std::uint16_t stored = 0;
constexpr std::uint16_t deflated = 8;
constexpr std::uint16_t zip_version = 20;
constexpr std::uint16_t zip_date = 0x21; // 1980-01-01

struct Member {
    std::string_view name;
    std::size_t limit;
};

constexpr std::array members{
    Member{"project.json", project_document_limit},
    Member{"cloud.ply", project_cloud_limit},
    Member{"results.csv", project_csv_limit},
};

[[noreturn]] void bad_archive()
{
    throw std::invalid_argument("Archivo de proyecto dañado o no compatible");
}

const json& field(const json& object, std::string_view key)
{
    static const json missing;
    const auto it = object.find(key);
    return it == object.end() ? missing : *it;
}

bool number_vector(const json& value, std::size_t count)
{
    return value.is_array() && value.size() == count &&
           std::all_of(value.begin(), value.end(), [](const json& item) { return item.is_number(); });
}

bool in_range(const json& value, double low, double high)
{
    return value.is_number() && value.get<double>() >= low && value.get<double>() <= high;
}

bool terrain_class(const json& value)
{
    constexpr std::array<std::string_view, 6> classes{
        "desconocido", "grava", "roca", "plantas", "barro", "tierra"};
    return value.is_string() &&
           std::find(classes.begin(), classes.end(), value.get_ref<const std::string&>()) !=
               classes.end();
}

void validate_document(const json& value, int depth)
{
    if (depth > 30) {
        throw std::invalid_argument("Documento demasiado anidado");
    }
    if (value.is_number_float() && !std::isfinite(value.get<double>())) {
        throw std::invalid_argument("El documento contiene números no finitos");
    }
    if (value.is_array() || value.is_object()) {
        for (const auto& child : value) {
            validate_document(child, depth + 1);
        }
    }
}

void validate_frame(const json& frame)
{
    for (const auto key : {"time", "tracking_error", "speed_error",
                           "tangential_acceleration", "acceleration_error"}) {
        if (frame.contains(key) && !frame[key].is_number()) {
            throw std::invalid_argument("Métrica de simulación inválida");
        }
    }
    for (const auto key : {"points", "normal", "friction", "moments"}) {
        const auto& vectors = field(frame, key);
        if (frame.contains(key) && (!vectors.is_array() ||
            !std::all_of(vectors.begin(), vectors.end(), [](const json& v) { return number_vector(v, 3); }))) {
            throw std::invalid_argument("Contactos de simulación inválidos");
        }
    }
    const auto& contacts = field(frame, "contact");
    if (frame.contains("contact") && (!contacts.is_array() ||
        !std::all_of(contacts.begin(), contacts.end(), [](const json& v) { return v.is_number(); }))) {
        throw std::invalid_argument("Contactos de simulación inválidos");
    }
    for (const auto& [key, size] : {std::pair{"position", 3}, {"orientation", 4}, {"velocity", 3}}) {
        if (frame.contains(key) && !number_vector(frame[key], size)) {
            throw std::invalid_argument(std::string("Estado de simulación inválido: ") + key);
        }
    }
}

void validate_terrain(const json& terrain)
{
    if ((terrain.contains("classes") && !terrain["classes"].is_object()) ||
        (terrain.contains("zones") && !terrain["zones"].is_array())) {
        throw std::invalid_argument("Terreno de proyecto inválido");
    }
    for (const auto& [name, properties] : field(terrain, "classes").items()) {
        if (!terrain_class(name) || !properties.is_object()) {
            throw std::invalid_argument("Clase de terreno inválida");
        }
        if (!in_range(field(properties, "friction"), 0, 3) ||
            !in_range(field(properties, "stiffness"), 1000, 1e8)) {
            throw std::invalid_argument("Propiedad de terreno inválida");
        }
    }
    for (const auto& zone : field(terrain, "zones")) {
        const auto& radius = field(zone, "radius");
        if (!zone.is_object() || !number_vector(field(zone, "center"), 3) ||
            !radius.is_number() || radius.get<double>() <= 0 ||
            !terrain_class(field(zone, "class"))) {
            throw std::invalid_argument("Zona de terreno inválida");
        }
    }
}

bool canonical_integer(std::string_view value)
{
    if (value == "0") {
        return true;
    }
    if (!value.empty() && value.front() == '-') {
        value.remove_prefix(1);
    }
    return !value.empty() && value.front() != '0' &&
           std::all_of(value.begin(), value.end(), [](char c) { return c >= '0' && c <= '9'; });
}

std::string read_bounded(const std::filesystem::path& path, std::size_t maximum)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        throw std::invalid_argument("No se pudo leer un archivo del proyecto");
    }
    std::string result;
    std::array<char, 64 * 1024> buffer;
    while (stream) {
        const auto remaining = maximum - result.size() + 1;
        stream.read(buffer.data(), static_cast<std::streamsize>(std::min(remaining, buffer.size())));
        result.append(buffer.data(), static_cast<std::size_t>(stream.gcount()));
        if (result.size() > maximum) {
            break;
        }
    }
    if (stream.bad() || result.empty() || result.size() > maximum) {
        throw std::invalid_argument("Archivo vacío o demasiado grande para el proyecto");
    }
    return result;
}

template <std::size_t width>
std::uint32_t read_le(std::string_view bytes, std::size_t offset)
{
    if (offset > bytes.size() || bytes.size() - offset < width) {
        bad_archive();
    }
    std::uint32_t result = 0;
    for (std::size_t i = 0; i < width; ++i) {
        result |= static_cast<std::uint32_t>(static_cast<unsigned char>(bytes[offset + i])) << (8 * i);
    }
    return result;
}

template <std::size_t width>
void append_le(std::string& bytes, std::uint32_t value)
{
    for (std::size_t i = 0; i < width; ++i) {
        bytes.push_back(static_cast<char>(value >> (8 * i)));
    }
}

std::uint32_t crc(std::string_view bytes)
{
    return static_cast<std::uint32_t>(crc32(0, reinterpret_cast<const Bytef*>(bytes.data()),
                                            static_cast<uInt>(bytes.size())));
}

struct ZipEntry {
    std::string_view name;
    std::uint32_t size;
    std::uint32_t compressed_size;
    std::uint32_t checksum;
    std::uint32_t offset;
    std::uint16_t method;
    std::uint16_t flags;
};

void append_header(std::string& output, const ZipEntry& entry, bool central)
{
    append_le<4>(output, central ? directory_signature : local_signature);
    if (central) {
        append_le<2>(output, zip_version);
    }
    append_le<2>(output, zip_version);
    append_le<2>(output, entry.flags);
    append_le<2>(output, entry.method);
    append_le<2>(output, 0);
    append_le<2>(output, zip_date);
    append_le<4>(output, entry.checksum);
    append_le<4>(output, entry.compressed_size);
    append_le<4>(output, entry.size);
    append_le<2>(output, static_cast<std::uint32_t>(entry.name.size()));
    append_le<2>(output, 0);
    if (central) {
        append_le<2>(output, 0);
        append_le<2>(output, 0);
        append_le<2>(output, 0);
        append_le<4>(output, 0);
        append_le<4>(output, entry.offset);
    }
    output.append(entry.name);
}

std::string compress(std::string_view data)
{
    std::string compressed(compressBound(static_cast<uLong>(data.size())), '\0');
    z_stream stream{};
    if (deflateInit2(&stream, Z_DEFAULT_COMPRESSION, Z_DEFLATED, -MAX_WBITS, 8,
                     Z_DEFAULT_STRATEGY) != Z_OK) {
        throw std::runtime_error("No se pudo comprimir el proyecto");
    }
    stream.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(data.data()));
    stream.avail_in = static_cast<uInt>(data.size());
    stream.next_out = reinterpret_cast<Bytef*>(compressed.data());
    stream.avail_out = static_cast<uInt>(compressed.size());
    const auto status = deflate(&stream, Z_FINISH);
    const auto count = stream.total_out;
    deflateEnd(&stream);
    if (status != Z_STREAM_END) {
        throw std::runtime_error("No se pudo comprimir el proyecto");
    }
    compressed.resize(count);
    return compressed;
}

std::string encode_zip(const std::array<std::string_view, 3>& data)
{
    std::string output;
    std::array<ZipEntry, 3> entries{};
    const std::uint32_t count = data[2].empty() ? 2 : 3;
    for (std::uint32_t i = 0; i < count; ++i) {
        const auto compressed = compress(data[i]);
        entries[i] = {members[i].name, static_cast<std::uint32_t>(data[i].size()),
                      static_cast<std::uint32_t>(compressed.size()), crc(data[i]),
                      static_cast<std::uint32_t>(output.size()), deflated, 0};
        append_header(output, entries[i], false);
        output.append(compressed);
    }
    const auto central_start = static_cast<std::uint32_t>(output.size());
    for (std::uint32_t i = 0; i < count; ++i) {
        append_header(output, entries[i], true);
    }
    const auto central_size = static_cast<std::uint32_t>(output.size()) - central_start;
    append_le<4>(output, end_signature);
    append_le<2>(output, 0);
    append_le<2>(output, 0);
    append_le<2>(output, count);
    append_le<2>(output, count);
    append_le<4>(output, central_size);
    append_le<4>(output, central_start);
    append_le<2>(output, 0);
    return output;
}

std::size_t end_record(std::string_view raw)
{
    constexpr std::size_t end_size = 22;
    constexpr std::size_t max_comment = 65535;
    if (raw.size() < end_size) {
        bad_archive();
    }
    const std::size_t earliest = raw.size() - std::min(raw.size(), end_size + max_comment);
    for (std::size_t cursor = raw.size() - end_size + 1; cursor > earliest;) {
        --cursor;
        if (read_le<4>(raw, cursor) == end_signature &&
            read_le<2>(raw, cursor + 20) == raw.size() - cursor - end_size) {
            return cursor;
        }
    }
    bad_archive();
}

std::string extract_member(std::string_view body, const ZipEntry& entry)
{
    const auto offset = static_cast<std::size_t>(entry.offset);
    if (read_le<4>(body, offset) != local_signature ||
        read_le<2>(body, offset + 6) != entry.flags ||
        read_le<2>(body, offset + 8) != entry.method) {
        bad_archive();
    }
    const auto name_size = read_le<2>(body, offset + 26);
    const auto extra_size = read_le<2>(body, offset + 28);
    const auto start = offset + 30 + name_size + extra_size;
    if (start > body.size() || entry.compressed_size > body.size() - start ||
        body.substr(offset + 30, name_size) != entry.name) {
        bad_archive();
    }
    const auto compressed = body.substr(start, entry.compressed_size);
    if (entry.method == stored) {
        if (entry.size != compressed.size() || crc(compressed) != entry.checksum) {
            bad_archive();
        }
        return std::string(compressed);
    }
    if (entry.method != deflated) {
        bad_archive();
    }
    // One extra byte detects deflate data exceeding its declared uncompressed size.
    std::string data(static_cast<std::size_t>(entry.size) + 1, '\0');
    z_stream stream{};
    if (inflateInit2(&stream, -MAX_WBITS) != Z_OK) {
        bad_archive();
    }
    stream.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(compressed.data()));
    stream.avail_in = static_cast<uInt>(compressed.size());
    stream.next_out = reinterpret_cast<Bytef*>(data.data());
    stream.avail_out = static_cast<uInt>(data.size());
    const auto status = inflate(&stream, Z_FINISH);
    const auto count = stream.total_out;
    const auto remaining = stream.avail_in;
    inflateEnd(&stream);
    if (status != Z_STREAM_END || count != entry.size || remaining != 0) {
        bad_archive();
    }
    data.resize(count);
    if (crc(data) != entry.checksum) {
        bad_archive();
    }
    return data;
}

std::array<std::string, 3> decode_zip(std::string_view raw)
{
    const auto end = end_record(raw);
    const auto count = read_le<2>(raw, end + 10);
    const auto directory_size = read_le<4>(raw, end + 12);
    const auto directory_offset = read_le<4>(raw, end + 16);
    if (read_le<2>(raw, end + 4) != 0 || read_le<2>(raw, end + 6) != 0 ||
        read_le<2>(raw, end + 8) != count || directory_size > end ||
        directory_offset > end - directory_size) {
        bad_archive();
    }
    if (count < 2 || count > members.size()) {
        throw std::invalid_argument("Contenido del archivo de proyecto inválido");
    }
    const auto directory_begin = end - directory_size;
    const auto archive_start = directory_begin - directory_offset;
    const auto directory = raw.substr(directory_begin, directory_size);
    const auto body = raw.substr(archive_start, directory_offset);
    std::array<std::optional<ZipEntry>, 3> entries;
    std::size_t cursor = 0;
    for (std::uint32_t i = 0; i < count; ++i) {
        if (read_le<4>(directory, cursor) != directory_signature ||
            read_le<2>(directory, cursor + 34) != 0) {
            bad_archive();
        }
        const auto name_length = read_le<2>(directory, cursor + 28);
        const auto entry_length = 46u + name_length + read_le<2>(directory, cursor + 30) +
                                  read_le<2>(directory, cursor + 32);
        if (entry_length > directory.size() - cursor) {
            bad_archive();
        }
        const auto name = directory.substr(cursor + 46, name_length);
        const auto member = std::find_if(members.begin(), members.end(),
                                         [name](const Member& item) { return item.name == name; });
        if (member == members.end() || entries[member - members.begin()]) {
            throw std::invalid_argument("Contenido del archivo de proyecto inválido");
        }
        const auto flags = static_cast<std::uint16_t>(read_le<2>(directory, cursor + 8));
        const auto size = read_le<4>(directory, cursor + 24);
        if ((flags & 0x2041) || size == 0 || size > member->limit) {
            throw std::invalid_argument("Miembro del proyecto vacío, cifrado o demasiado grande");
        }
        entries[member - members.begin()] = ZipEntry{
            name, size, read_le<4>(directory, cursor + 20), read_le<4>(directory, cursor + 16),
            read_le<4>(directory, cursor + 42),
            static_cast<std::uint16_t>(read_le<2>(directory, cursor + 10)), flags};
        cursor += entry_length;
    }
    if (!entries[0] || !entries[1] || cursor != directory.size()) {
        throw std::invalid_argument("Contenido del archivo de proyecto inválido");
    }
    std::array<std::string, 3> data;
    for (std::size_t i = 0; i < data.size(); ++i) {
        if (!entries[i]) {
            continue;
        }
        std::size_t member_end = body.size();
        for (std::size_t j = 0; j < entries.size(); ++j) {
            if (i == j || !entries[j] || entries[j]->offset < entries[i]->offset) {
                continue;
            }
            member_end = std::min(member_end, static_cast<std::size_t>(entries[j]->offset));
        }
        data[i] = extract_member(body.substr(0, member_end), *entries[i]);
    }
    return data;
}

} // namespace

void validate_project(const nlohmann::json& project)
{
    if (!project.is_object()) {
        throw std::invalid_argument("Contenido de proyecto inválido");
    }
    validate_document(project, 0);
    if (project.contains("csv_available") && !project["csv_available"].is_boolean()) {
        throw std::invalid_argument("Estado CSV de proyecto inválido");
    }
    if (project.contains("playback_index") &&
        (!project["playback_index"].is_number_integer() || project["playback_index"] < 0)) {
        throw std::invalid_argument("Posición de reproducción inválida");
    }
    const auto& applied = field(project, "applied_pose");
    if (!applied.is_null() && (!applied.is_object() ||
        !number_vector(field(applied, "position"), 3) ||
        !number_vector(field(applied, "orientation"), 4))) {
        throw std::invalid_argument("Pose aplicada inválida");
    }
    for (const auto key : {"pose", "terrain"}) {
        if (project.contains(key) && !project[key].is_object()) {
            throw std::invalid_argument(std::string("Campo de proyecto inválido: ") + key);
        }
    }
    if (project.contains("vehicle")) {
        const auto& vehicle = project["vehicle"];
        if (vehicle != "MAN_5t" && vehicle != "MAN_7t" && vehicle != "MAN_10t") {
            throw std::invalid_argument("Vehículo de proyecto no compatible");
        }
    }
    for (const auto& value : field(project, "pose")) {
        if (!value.is_number()) {
            throw std::invalid_argument("Pose de proyecto inválida");
        }
    }
    for (const auto key : {"route", "frames", "trajectory"}) {
        const auto& values = field(project, key);
        if (project.contains(key) && (!values.is_array() ||
            !std::all_of(values.begin(), values.end(), [](const json& item) { return item.is_object(); }))) {
            throw std::invalid_argument(std::string("Campo de proyecto inválido: ") + key);
        }
    }
    const auto& route = field(project, "route");
    if (route.size() > 10000) {
        throw std::invalid_argument("Demasiados puntos de trayectoria");
    }
    for (const auto& point : route) {
        for (const auto key : {"x", "y", "z", "speed", "acceleration"}) {
            if (!field(point, key).is_number()) {
                throw std::invalid_argument("Punto de trayectoria inválido");
            }
        }
        if (!in_range(point["speed"], 0, 100) || !in_range(point["acceleration"], -20, 20)) {
            throw std::invalid_argument("Objetivo de trayectoria inválido");
        }
    }
    for (const auto key : {"frames", "trajectory"}) {
        for (const auto& frame : field(project, key)) {
            validate_frame(frame);
        }
    }
    if (project.contains("warnings") && !project["warnings"].is_string()) {
        throw std::invalid_argument("Avisos de proyecto inválidos");
    }
    validate_terrain(field(project, "terrain"));
}

std::string export_archive(nlohmann::json project, nlohmann::json cloud,
                           const std::filesystem::path& csv_path)
{
    validate_project(project);
    const auto& path = field(cloud, "path");
    if (!cloud.is_object() || !path.is_string() || path.get_ref<const std::string&>().empty()) {
        throw std::invalid_argument("Carga primero una nube");
    }
    const auto& path_bytes = path.get_ref<const std::string&>();
    const auto source = read_bounded(std::filesystem::path(std::u8string(path_bytes.begin(), path_bytes.end())),
                                     project_cloud_limit);
    if (field(cloud, "sha256") != sha256(source)) {
        throw std::invalid_argument("La nube original ha cambiado; vuelve a cargarla antes de guardar");
    }
    const bool include_csv = !csv_path.empty() && field(project, "csv_available") == true;
    project.erase("config");
    project.erase("configuration");
    cloud.erase("path");
    project["cloud"] = std::move(cloud);
    validate_document(project["cloud"], 0);
    const json document{{"format", archive_format}, {"version", archive_version},
                        {"project", std::move(project)}};
    const auto encoded = document.dump();
    if (encoded.size() > project_document_limit) {
        throw std::invalid_argument("El estado del proyecto supera 64 MB");
    }
    const auto csv = include_csv ? read_bounded(csv_path, project_csv_limit) : std::string{};
    return encode_zip({encoded, source, csv});
}

PortableProject read_archive(std::string_view raw)
{
    if (raw.empty() || raw.size() > project_archive_limit) {
        throw std::invalid_argument("Tamaño de proyecto inválido");
    }
    auto data = decode_zip(raw);
    if (data[0].find('\0') != std::string::npos) {
        bad_archive();
    }
    json document;
    try {
        document = json::parse(data[0], [](int depth, json::parse_event_t, json&) {
            if (depth > 32) {
                throw std::invalid_argument("Documento demasiado anidado");
            }
            return true;
        });
    } catch (const json::exception&) {
        bad_archive();
    }
    if (field(document, "format") != archive_format || field(document, "version") != archive_version) {
        throw std::invalid_argument("Formato o versión de proyecto no compatible");
    }
    auto project = std::move(document["project"]);
    validate_project(project);
    const auto& reference = field(project, "cloud");
    if (!reference.is_object()) {
        throw std::invalid_argument("Faltan los datos de la nube en el proyecto");
    }
    const auto& mapping = field(reference, "label_mapping");
    if (reference.contains("label_mapping") && !mapping.is_object()) {
        throw std::invalid_argument("Tabla de clases de proyecto inválida");
    }
    for (const auto& [key, value] : mapping.items()) {
        if (!canonical_integer(key) || !terrain_class(value)) {
            throw std::invalid_argument("Tabla de clases de proyecto inválida");
        }
    }
    if (field(reference, "sha256") != sha256(data[1])) {
        throw std::invalid_argument("La huella de la nube no coincide con el proyecto");
    }
    return {std::move(project), std::move(data[1]),
            data[2].empty() ? std::nullopt : std::make_optional(std::move(data[2]))};
}

} // namespace fusionforce
