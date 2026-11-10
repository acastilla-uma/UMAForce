#include "fusionforce/cloud_store.hpp"
#include "fusionforce/sha256.hpp"

#include <nlohmann/json.hpp>
#include <sqlite3.h>

#include <algorithm>
#include <bit>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <stdexcept>

namespace fusionforce {
namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;
constexpr double inf = std::numeric_limits<double>::infinity();
constexpr std::uint64_t import_block = 100000, overview_limit = 30000;

[[noreturn]] void fail(const std::string& message) { throw std::runtime_error(message); }

std::string utf8(const fs::path& path) {
    auto text = path.u8string();
    return {text.begin(), text.end()};
}

using Db = std::unique_ptr<sqlite3, decltype(&sqlite3_close)>;

void checked(sqlite3* db, int code) {
    if (code != SQLITE_OK) fail(sqlite3_errmsg(db));
}

Db connect(const fs::path& path, int flags) {
    sqlite3* raw = nullptr;
    const auto code = sqlite3_open_v2(utf8(path).c_str(), &raw, flags, nullptr);
    Db db(raw, sqlite3_close);
    if (!db) fail("No se pudo abrir el índice de nube");
    checked(db.get(), code);
    return db;
}

void sql(sqlite3* db, const char* command) {
    checked(db, sqlite3_exec(db, command, nullptr, nullptr, nullptr));
}

class Statement {
public:
    Statement(sqlite3* db, const char* command) : db_(db) {
        sqlite3_stmt* stmt = nullptr;
        const auto code = sqlite3_prepare_v2(db, command, -1, &stmt, nullptr);
        stmt_.reset(stmt);
        checked(db, code);
    }
    void number(int i, double value) { checked(db_, sqlite3_bind_double(stmt_.get(), i, value)); }
    void integer(int i, std::int64_t value) { checked(db_, sqlite3_bind_int64(stmt_.get(), i, value)); }
    void blob(int i, std::string_view value) {
        checked(db_, sqlite3_bind_blob64(stmt_.get(), i, value.data(), value.size(), SQLITE_TRANSIENT));
    }
    void text(int i, std::string_view value) {
        checked(db_, sqlite3_bind_text64(stmt_.get(), i, value.data(), value.size(), SQLITE_TRANSIENT, SQLITE_UTF8));
    }
    bool next() {
        const auto code = sqlite3_step(stmt_.get());
        if (code == SQLITE_ROW) return true;
        if (code != SQLITE_DONE) checked(db_, code);
        return false;
    }
    void write() {
        if (next()) fail("Fila inesperada en escritura");
        checked(db_, sqlite3_reset(stmt_.get()));
    }
    std::int64_t integer(int i) const { return sqlite3_column_int64(stmt_.get(), i); }
    std::string_view text(int i) const {
        const auto* data = reinterpret_cast<const char*>(sqlite3_column_text(stmt_.get(), i));
        return data ? std::string_view(data, sqlite3_column_bytes(stmt_.get(), i)) : std::string_view{};
    }
    std::string_view blob(int i) const {
        const auto* data = static_cast<const char*>(sqlite3_column_blob(stmt_.get(), i));
        return data ? std::string_view(data, sqlite3_column_bytes(stmt_.get(), i)) : std::string_view{};
    }
private:
    sqlite3* db_;
    std::unique_ptr<sqlite3_stmt, decltype(&sqlite3_finalize)> stmt_{nullptr, sqlite3_finalize};
};

bool read_line(std::istream& stream, std::string& line) {
    line.clear();
    for (char ch; stream.get(ch);) {
        if (line.size() == 65536) fail("Línea PLY demasiado larga");
        if (ch == '\n') return true;
        line.push_back(ch);
    }
    return !line.empty();
}

void split(std::string_view line, std::vector<std::string_view>& fields) {
    constexpr std::string_view whitespace = " \t\r\v\f";
    fields.clear();
    for (auto pos = line.find_first_not_of(whitespace); pos != std::string_view::npos;
         pos = line.find_first_not_of(whitespace, pos)) {
        const auto end = line.find_first_of(whitespace, pos);
        fields.push_back(line.substr(pos, end == std::string_view::npos ? end : end - pos));
        pos = end;
    }
}

template<class T>
T parse(std::string_view token) {
    if (token.starts_with('+')) token.remove_prefix(1);
    T value{};
    const auto [end, status] = std::from_chars(token.data(), token.data() + token.size(), value);
    if (status != std::errc{} || end != token.data() + token.size()) fail("Datos PLY ASCII inválidos");
    return value;
}

enum class Kind { signed_integer, unsigned_integer, floating };
struct Type { Kind kind; unsigned size; };
struct Property { std::string name; Type type; };

struct Header {
    bool ascii = false, big_endian = false;
    std::uint64_t count = 0;
    std::vector<Property> properties;
    std::array<std::size_t, 3> xyz{};
    std::optional<std::size_t> label;
};

const std::map<std::string_view, Type> types{
    {"char", {Kind::signed_integer, 1}}, {"int8", {Kind::signed_integer, 1}},
    {"uchar", {Kind::unsigned_integer, 1}}, {"uint8", {Kind::unsigned_integer, 1}},
    {"short", {Kind::signed_integer, 2}}, {"int16", {Kind::signed_integer, 2}},
    {"ushort", {Kind::unsigned_integer, 2}}, {"uint16", {Kind::unsigned_integer, 2}},
    {"int", {Kind::signed_integer, 4}}, {"int32", {Kind::signed_integer, 4}},
    {"uint", {Kind::unsigned_integer, 4}}, {"uint32", {Kind::unsigned_integer, 4}},
    {"float", {Kind::floating, 4}}, {"float32", {Kind::floating, 4}},
    {"double", {Kind::floating, 8}}, {"float64", {Kind::floating, 8}}
};

Header read_header(std::istream& stream) {
    Header header;
    std::string line, element, format;
    std::vector<std::string_view> fields;
    if (!read_line(stream, line)) fail("No es un archivo PLY");
    split(line, fields);
    if (fields != std::vector<std::string_view>{"ply"}) fail("No es un archivo PLY");
    bool vertex = false, complete = false;
    for (int row = 0; row < 10000 && read_line(stream, line); ++row) {
        if (std::ranges::any_of(line, [](unsigned char ch) { return ch > 127; })) fail("Cabecera PLY inválida");
        split(line, fields);
        if (fields.empty()) continue;
        if (fields[0] == "format") {
            if (fields.size() != 3 || fields[2] != "1.0") fail("Versión PLY no soportada");
            format = fields[1];
        } else if (fields[0] == "element") {
            if (fields.size() != 3) fail("Elemento PLY inválido");
            element = fields[1];
            const auto count = parse<std::uint64_t>(fields[2]);
            if (element == "vertex") {
                if (vertex) fail("Elemento vertex repetido");
                header.count = count;
                vertex = true;
            } else if (!vertex && count != 0) {
                fail("El elemento vertex debe ser el primero del PLY");
            }
        } else if (fields[0] == "property" && element == "vertex") {
            if (fields.size() != 3 || !types.contains(fields[1])) {
                fail("Solo se admiten propiedades escalares numéricas por vértice");
            }
            if (std::ranges::find(header.properties, fields[2], &Property::name) != header.properties.end()) {
                fail("Propiedad PLY repetida");
            }
            header.properties.push_back({std::string(fields[2]), types.at(fields[1])});
        } else if (fields[0] == "end_header") {
            complete = true;
            break;
        }
    }
    if (!complete) fail("Cabecera PLY incompleta o demasiado larga");
    header.ascii = format == "ascii";
    header.big_endian = format == "binary_big_endian";
    if (!header.ascii && !header.big_endian && format != "binary_little_endian") fail("Formato PLY no soportado");
    const auto index = [&](std::string_view name) -> std::optional<std::size_t> {
        const auto found = std::ranges::find(header.properties, name, &Property::name);
        if (found == header.properties.end()) return {};
        return std::size_t(found - header.properties.begin());
    };
    unsigned axis = 0;
    for (auto name : {"x", "y", "z"}) {
        const auto property = index(name);
        if (header.count < 3 || !property) fail("Se requieren al menos tres vértices XYZ");
        header.xyz[axis++] = *property;
    }
    for (auto name : {"classification", "class", "label"}) {
        header.label = index(name);
        if (header.label) break;
    }
    return header;
}

std::uint64_t read_bits(std::string_view bytes, bool big_endian = false) {
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        value = (value << 8) | static_cast<unsigned char>(bytes[big_endian ? i : bytes.size() - 1 - i]);
    }
    return value;
}

double binary(Type type, std::uint64_t bits) {
    if (type.kind == Kind::floating) {
        return type.size == 4 ? std::bit_cast<float>(std::uint32_t(bits)) : std::bit_cast<double>(bits);
    }
    const auto mask = (std::uint64_t(1) << (8 * type.size)) - 1;
    bits &= mask;
    if (type.kind == Kind::signed_integer && bits > mask / 2) return -double(mask - bits + 1);
    return double(bits);
}

double ascii(Type type, std::string_view value) {
    if (type.kind == Kind::floating) return type.size == 4 ? parse<float>(value) : parse<double>(value);
    const auto bits = value.starts_with('-') ? std::uint64_t(parse<std::int64_t>(value)) : parse<std::uint64_t>(value);
    return binary(type, bits);
}

void append_bits(std::string& bytes, std::uint64_t bits) {
    for (unsigned i = 0; i < 8; ++i) bytes.push_back(static_cast<char>((bits >> (i * 8)) & 255));
}

void save_points(Statement& stmt, int column, std::span<const CloudPoint> points) {
    std::string xyz, labels;
    xyz.reserve(points.size() * 24);
    labels.reserve(points.size() * 8);
    for (const auto& point : points) {
        for (double x : point.xyz) append_bits(xyz, std::bit_cast<std::uint64_t>(x));
        append_bits(labels, std::bit_cast<std::uint64_t>(point.label));
    }
    stmt.blob(column, xyz);
    stmt.blob(column + 1, labels);
    stmt.write();
}

template<class Visitor>
void decode(const Statement& stmt, Visitor visit) {
    const auto xyz = stmt.blob(0), labels = stmt.blob(1);
    if (xyz.size() % 24 || xyz.size() / 3 != labels.size()) fail("Bloque de nube inválido");
    for (std::size_t i = 0; i < xyz.size(); i += 24) {
        CloudPoint point{};
        for (unsigned axis = 0; axis < 3; ++axis) {
            point.xyz[axis] = std::bit_cast<double>(read_bits(xyz.substr(i + axis * 8, 8)));
        }
        point.label = std::bit_cast<std::int64_t>(read_bits(labels.substr(i / 3, 8)));
        visit(point);
    }
}

template<std::size_t N>
bool finite(const std::array<double, N>& values) {
    return std::ranges::all_of(values, [](double x) { return std::isfinite(x); });
}

void expand(std::array<CloudXyz, 2>& bounds, const CloudXyz& point) {
    for (unsigned axis = 0; axis < 3; ++axis) {
        bounds[0][axis] = std::min(bounds[0][axis], point[axis]);
        bounds[1][axis] = std::max(bounds[1][axis], point[axis]);
    }
}

void check_cancel(const CloudOptions& options) {
    if (options.cancel && options.cancel()) fail("Indexación cancelada; el archivo original no se ha modificado");
}

CloudLabels load_labels(const fs::path& source, const std::optional<CloudLabels>& specified) {
    auto sidecar = source;
    sidecar += ".labels.json";
    CloudLabels labels;
    if (specified) {
        labels = *specified;
    } else if (fs::is_regular_file(sidecar)) {
        if (fs::file_size(sidecar) > 1000000) fail("Tabla de etiquetas demasiado grande");
        std::ifstream stream(sidecar, std::ios::binary);
        labels = Json::parse(stream).get<CloudLabels>();
    }
    constexpr std::array<std::string_view, 6> allowed{"grava", "roca", "plantas", "barro", "tierra", "desconocido"};
    for (const auto& [id, category] : labels) {
        auto digits = std::string_view(id);
        if (digits.starts_with('-')) digits.remove_prefix(1);
        const bool canonical = id == "0" || (!digits.empty() && digits.front() != '0' &&
            std::ranges::all_of(digits, [](char ch) { return ch >= '0' && ch <= '9'; }));
        if (!canonical || std::ranges::find(allowed, category) == allowed.end()) fail("Tabla de etiquetas inválida");
    }
    return labels;
}

std::string mapping_hash(const CloudLabels& labels) {
    std::string serialized = "{";
    for (const auto& [id, category] : labels) {
        if (serialized.size() > 1) serialized += ", ";
        serialized += "\"" + id + "\": \"" + category + "\"";
    }
    return sha256(serialized + "}").substr(0, 16);
}

void import_cloud(sqlite3* db, CloudMetadata& m, const CloudOptions& options) {
    sql(db, "PRAGMA journal_mode=OFF; PRAGMA synchronous=OFF; PRAGMA cache_size=-8192;"
            "CREATE TABLE metadata(key TEXT PRIMARY KEY,value TEXT);"
            "CREATE TABLE chunks(id INTEGER PRIMARY KEY,xyz BLOB NOT NULL,labels BLOB NOT NULL);"
            "CREATE VIRTUAL TABLE spatial USING rtree(id,xmin,xmax,ymin,ymax,zmin,zmax);"
            "CREATE TABLE overview(xyz BLOB NOT NULL,labels BLOB NOT NULL); BEGIN");
    Statement chunks(db, "INSERT INTO chunks VALUES(?,?,?)");
    Statement spatial(db, "INSERT INTO spatial VALUES(?,?,?,?,?,?,?)");
    std::ifstream stream(m.path, std::ios::binary);
    const auto header = read_header(stream);
    m.count = header.count;
    if (header.label) m.label_property = header.properties[*header.label].name;
    m.bounds = {{{inf, inf, inf}, {-inf, -inf, -inf}}};
    const auto stride = (header.count - 1) / overview_limit + 1;
    const auto tile = [](const CloudPoint& point) {
        return CloudXy{std::floor(point.xyz[0] / 10), std::floor(point.xyz[1] / 10)};
    };
    std::vector<CloudPoint> overview, block;
    std::vector<double> values(header.properties.size());
    std::vector<std::string_view> fields;
    std::size_t width = 0;
    for (const auto& property : header.properties) width += property.type.size;
    std::string record(width, '\0'), line;
    for (std::uint64_t start = 0; start < header.count;) {
        check_cancel(options);
        const auto size = std::min(import_block, header.count - start);
        block.clear();
        block.reserve(size);
        for (std::uint64_t row = 0; row < size; ++row) {
            if (header.ascii) {
                if (!read_line(stream, line)) fail("Fila PLY ASCII incompleta o incompatible");
                split(line, fields);
                if (fields.size() != values.size()) fail("Fila PLY ASCII incompleta o incompatible");
                for (std::size_t i = 0; i < values.size(); ++i) values[i] = ascii(header.properties[i].type, fields[i]);
            } else {
                if (!stream.read(record.data(), static_cast<std::streamsize>(width))) fail("Archivo PLY truncado");
                std::size_t offset = 0;
                for (std::size_t i = 0; i < values.size(); ++i) {
                    const auto type = header.properties[i].type;
                    values[i] = binary(type, read_bits(std::string_view(record).substr(offset, type.size), header.big_endian));
                    offset += type.size;
                }
            }
            CloudPoint point{{values[header.xyz[0]], values[header.xyz[1]], values[header.xyz[2]]}};
            if (!finite(point.xyz)) fail("Se requieren coordenadas XYZ finitas");
            if (header.label) {
                const auto label = values[*header.label];
                if (!std::isfinite(label) || std::floor(label) != label || std::abs(label) > 0x1p53) {
                    fail("Las etiquetas deben ser IDs enteros finitos");
                }
                point.label = static_cast<std::int64_t>(label);
            }
            expand(m.bounds, point.xyz);
            if ((start + row) % stride == 0) overview.push_back(point);
            block.push_back(point);
        }
        std::ranges::stable_sort(block, {}, tile);
        const std::span<const CloudPoint> points(block);
        for (std::size_t first = 0; first < points.size();) {
            auto last = first + 1;
            auto bounds = std::array{points[first].xyz, points[first].xyz};
            const auto id = tile(points[first]);
            for (; last < points.size() && tile(points[last]) == id; ++last) expand(bounds, points[last].xyz);
            chunks.integer(1, ++m.chunks);
            save_points(chunks, 2, points.subspan(first, last - first));
            spatial.integer(1, m.chunks);
            for (int axis = 0; axis < 3; ++axis) {
                spatial.number(2 + axis * 2, bounds[0][axis]);
                spatial.number(3 + axis * 2, bounds[1][axis]);
            }
            spatial.write();
            first = last;
        }
        start += size;
        if (options.progress) options.progress("index", start, header.count);
    }
    for (unsigned axis = 0; axis < 3; ++axis) m.center[axis] = (m.bounds[0][axis] + m.bounds[1][axis]) / 2;
    Statement sample(db, "INSERT INTO overview VALUES(?,?)");
    save_points(sample, 1, overview);
}

void validate_index(sqlite3* db, const CloudMetadata& m) {
    constexpr auto max_count = std::uint64_t(std::numeric_limits<std::int64_t>::max()) / 24;
    if (m.count < 3 || m.count > max_count || m.chunks == 0 || m.chunks > m.count) {
        fail("Índice incompleto: recuentos inválidos");
    }
    Statement chunks(db, "SELECT count(*),coalesce(sum(length(xyz)),0),"
                         "coalesce(sum(CASE WHEN typeof(xyz)!='blob' OR typeof(labels)!='blob' "
                         "OR length(xyz)=0 OR length(xyz)%24!=0 OR length(labels)*3!=length(xyz) THEN 1 ELSE 0 END),0) FROM chunks");
    if (!chunks.next() || chunks.integer(0) != static_cast<std::int64_t>(m.chunks) ||
        chunks.integer(1) != static_cast<std::int64_t>(m.count * 24) || chunks.integer(2)) {
        fail("Índice incompleto: faltan bloques o datos de puntos");
    }
    Statement spatial(db, "SELECT (SELECT count(*) FROM spatial),"
                          "(SELECT count(*) FROM chunks c LEFT JOIN spatial s ON c.id=s.id WHERE s.id IS NULL)");
    if (!spatial.next() || spatial.integer(0) != static_cast<std::int64_t>(m.chunks) || spatial.integer(1)) {
        fail("Índice incompleto: bloques espaciales inconsistentes");
    }
    Statement overview(db, "SELECT length(xyz),length(labels),typeof(xyz),typeof(labels) FROM overview");
    if (!overview.next() || overview.text(2) != "blob" || overview.text(3) != "blob" ||
        overview.integer(0) % 24 != 0 || overview.integer(0) < 24 ||
        std::uint64_t(overview.integer(0) / 24) > std::min(m.count, overview_limit) ||
        overview.integer(1) * 3 != overview.integer(0) || overview.next()) {
        fail("Índice incompleto: vista global inválida");
    }
}

template<class Visitor>
void visit_chunks(const fs::path& index, CloudXyz low, CloudXyz high, Visitor visit) {
    auto db = connect(index, SQLITE_OPEN_READONLY);
    Statement stmt(db.get(), "SELECT c.xyz,c.labels FROM spatial s JOIN chunks c ON c.id=s.id "
                             "WHERE s.xmax>=? AND s.xmin<=? AND s.ymax>=? AND s.ymin<=? "
                             "AND s.zmax>=? AND s.zmin<=? ORDER BY c.id");
    for (int axis = 0; axis < 3; ++axis) {
        stmt.number(axis * 2 + 1, low[axis]);
        stmt.number(axis * 2 + 2, high[axis]);
    }
    while (stmt.next()) visit(stmt);
}

bool within(const CloudXyz& point, const CloudXyz& low, const CloudXyz& high) {
    for (unsigned axis = 0; axis < 3; ++axis) {
        if (point[axis] < low[axis] || point[axis] > high[axis]) return false;
    }
    return true;
}

std::array<CloudXyz, 2> sphere_bounds(CloudXyz center, double radius) {
    if (!finite(center) || !std::isfinite(radius) || radius <= 0) fail("Centro o radio esférico inválido");
    return {{{center[0] - radius, center[1] - radius, center[2] - radius},
             {center[0] + radius, center[1] + radius, center[2] + radius}}};
}

double square(double x) { return x * x; }

double squared_distance(CloudXyz a, CloudXyz b) {
    return square(a[0] - b[0]) + square(a[1] - b[1]) + square(a[2] - b[2]);
}

class OverviewRandom {
public:
    std::uint64_t next() {
        // PCG64 state and increment after NumPy default_rng(0) seeding.
        const auto multiplier = wide(0x2360ed051fc65da4, 0x4385df649fccf645);
        const auto increment = wide(0x418ddadb3af71a82, 0x588133bc447873a9);
        state_ = state_ * multiplier + increment;
        const auto high = static_cast<std::uint64_t>(state_ >> 64);
        return std::rotr(high ^ static_cast<std::uint64_t>(state_), static_cast<int>(high >> 58)) >> 11;
    }
private:
    using UInt128 = __uint128_t;
    static constexpr UInt128 wide(std::uint64_t hi, std::uint64_t lo) { return (UInt128(hi) << 64) | lo; }
    UInt128 state_ = wide(0x1aa1b5345996452d, 0x09585eb7a69561e3);
};

template<class Predicate>
std::vector<CloudPoint> query(const fs::path& index, CloudXyz low, CloudXyz high,
                              std::optional<std::size_t> limit, Predicate accept) {
    if (limit && *limit == 0) fail("Límite de consulta inválido");
    std::vector<CloudPoint> result;
    visit_chunks(index, low, high, [&](const Statement& stmt) {
        decode(stmt, [&](const CloudPoint& point) {
            if (!within(point.xyz, low, high) || !accept(point)) return;
            if (limit && result.size() == *limit) {
                fail("Densidad de nube superior al límite de consulta (" + std::to_string(*limit) +
                     " puntos); reduzca la región explícitamente");
            }
            result.push_back(point);
        });
    });
    return result;
}

} // namespace

Json cloud_metadata_json(const CloudMetadata& m) {
    return {{"version", m.version}, {"complete", m.complete}, {"count", m.count}, {"path", utf8(m.path)},
            {"sha256", m.sha256}, {"bounds", m.bounds}, {"center", m.center},
            {"label_property", m.label_property ? Json(*m.label_property) : Json(nullptr)},
            {"label_mapping", m.label_mapping}, {"coordinate_frame", m.coordinate_frame},
            {"source_size", m.source_size}, {"source_mtime_ns", m.source_mtime_ns},
            {"index_seconds", m.index_seconds}, {"chunks", m.chunks}};
}

CloudStore::CloudStore(fs::path index_path) : index_path_(std::move(index_path)) {
    auto db = connect(index_path_, SQLITE_OPEN_READONLY);
    Statement stmt(db.get(), "SELECT value FROM metadata WHERE key='manifest'");
    if (!stmt.next()) fail("Índice incompleto");
    const auto data = Json::parse(stmt.text(0));
    if (data.at("version") != CloudMetadata::version || data.at("complete") != true) {
        fail("Índice incompatible o incompleto");
    }
    auto& m = metadata_;
    const auto path = data.at("path").get<std::string>();
    m.path = fs::path(std::u8string(path.begin(), path.end()));
    m.sha256 = data.at("sha256").get<std::string>();
    m.bounds = data.at("bounds").get<decltype(m.bounds)>();
    m.center = data.at("center").get<CloudXyz>();
    if (!data.at("label_property").is_null()) m.label_property = data.at("label_property").get<std::string>();
    m.label_mapping = data.at("label_mapping").get<CloudLabels>();
    m.count = data.at("count").get<std::uint64_t>();
    m.source_size = data.at("source_size").get<std::uint64_t>();
    m.source_mtime_ns = data.at("source_mtime_ns").get<std::int64_t>();
    m.index_seconds = data.at("index_seconds").get<double>();
    m.chunks = data.at("chunks").get<std::uint64_t>();
    validate_index(db.get(), m);
}

CloudStore CloudStore::open(const fs::path& path, const CloudOptions& options) {
    CloudMetadata m;
    m.path = fs::canonical(path);
    auto extension = m.path.extension().string();
    std::ranges::transform(extension, extension.begin(), [](unsigned char ch) { return std::tolower(ch); });
    if (!fs::is_regular_file(m.path) || extension != ".ply") fail("Selecciona un archivo PLY local");
    m.label_mapping = load_labels(m.path, options.label_mapping);
    m.source_size = fs::file_size(m.path);
    const auto modified = fs::last_write_time(m.path);
    m.source_mtime_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::file_clock::to_sys(modified).time_since_epoch()).count();
    const auto verify_source = [&] {
        if (fs::file_size(m.path) != m.source_size || fs::last_write_time(m.path) != modified) {
            fail("La nube cambió durante la indexación; vuelve a importarla");
        }
    };
    fs::create_directories(options.cache_dir);
    {
        Sha256 hash;
        std::ifstream stream(m.path, std::ios::binary);
        if (!stream) fail("No se pudo leer el archivo PLY");
        std::string buffer(4 * 1024 * 1024, '\0');
        std::uint64_t processed = 0;
        do {
            check_cancel(options);
            stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            const auto bytes = static_cast<std::size_t>(stream.gcount());
            if (bytes == 0) break;
            hash.update(std::string_view(buffer).substr(0, bytes));
            processed += bytes;
            if (options.progress) options.progress("fingerprint", processed, m.source_size);
        } while (stream);
        if (!stream.eof()) fail("No se pudo leer el archivo PLY");
        m.sha256 = hash.finish();
    }
    verify_source();
    check_cancel(options);
    const auto target = options.cache_dir / ("v1-" + m.sha256 + "-" + mapping_hash(m.label_mapping) + ".sqlite");
    if (fs::exists(target)) {
        std::optional<CloudStore> existing;
        try {
            existing.emplace(target);
        } catch (const Json::exception&) {
            // Rebuild damaged indexes from the original file.
        } catch (const std::runtime_error&) {
        }
        if (existing) {
            verify_source();
            check_cancel(options);
            existing->metadata_.path = m.path;
            return std::move(*existing);
        }
    }
    std::random_device random;
    auto temporary = target;
    temporary += "." + std::to_string(random()) + "-" + std::to_string(random()) + ".partial";
    try {
        const auto started = std::chrono::steady_clock::now();
        {
            auto db = connect(temporary, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_EXCLUSIVE);
            import_cloud(db.get(), m, options);
            verify_source();
            m.index_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            Statement row(db.get(), "INSERT INTO metadata VALUES('manifest',?)");
            row.text(1, cloud_metadata_json(m).dump());
            row.write();
            sql(db.get(), "COMMIT");
        }
        check_cancel(options);
        CloudStore store(temporary);
        fs::rename(temporary, target);
        store.index_path_ = target;
        return store;
    } catch (...) {
        std::error_code ignored;
        fs::remove(temporary, ignored);
        throw;
    }
}

std::vector<CloudPoint> CloudStore::all_points() const {
    auto db = connect(index_path_, SQLITE_OPEN_READONLY);
    Statement stmt(db.get(), "SELECT xyz,labels FROM chunks ORDER BY id");
    std::vector<CloudPoint> result;
    result.reserve(metadata_.count);
    while (stmt.next()) decode(stmt, [&](const CloudPoint& point) { result.push_back(point); });
    return result;
}

std::vector<CloudPoint> CloudStore::overview(std::size_t limit) const {
    if (limit == 0) fail("Límite de vista inválido");
    auto db = connect(index_path_, SQLITE_OPEN_READONLY);
    Statement stmt(db.get(), "SELECT xyz,labels FROM overview");
    if (!stmt.next()) fail("Índice incompleto: vista global ausente");
    const auto step = (stmt.blob(0).size() / 24 - 1) / limit + 1;
    std::vector<CloudPoint> result;
    std::size_t i = 0;
    decode(stmt, [&](const CloudPoint& point) {
        if (i++ % step == 0) result.push_back(point);
    });
    return result;
}

std::vector<CloudPoint> CloudStore::sphere(CloudXyz center, double radius, std::optional<std::size_t> limit) const {
    const auto [low, high] = sphere_bounds(center, radius);
    return query(index_path_, low, high, limit, [&](const CloudPoint& point) {
        return squared_distance(point.xyz, center) <= square(radius);
    });
}

CloudOverview CloudStore::sphere_overview(CloudXyz center, double radius, std::size_t limit) const {
    const auto [low, high] = sphere_bounds(center, radius);
    if (limit == 0) fail("Límite de vista inválido");
    struct Sample {
        std::uint64_t priority;
        std::uint64_t order;
        CloudPoint point;
        bool operator<(const Sample& other) const { return priority < other.priority; }
    };
    OverviewRandom random;
    std::priority_queue<Sample> kept;
    CloudOverview result;
    visit_chunks(index_path_, low, high, [&](const Statement& stmt) {
        decode(stmt, [&](const CloudPoint& point) {
            if (squared_distance(point.xyz, center) > square(radius)) return;
            Sample sample{random.next(), result.total++, point};
            if (kept.size() < limit) {
                kept.push(sample);
            } else if (sample < kept.top()) {
                kept.pop();
                kept.push(sample);
            }
        });
    });
    std::vector<Sample> sorted;
    sorted.reserve(kept.size());
    while (!kept.empty()) {
        sorted.push_back(kept.top());
        kept.pop();
    }
    std::ranges::sort(sorted, {}, &Sample::order);
    result.points.reserve(sorted.size());
    for (const auto& sample : sorted) result.points.push_back(sample.point);
    return result;
}

std::vector<CloudPoint> CloudStore::region_xy(std::array<double, 4> bounds, std::optional<std::size_t> limit) const {
    if (!finite(bounds) || bounds[0] > bounds[2] || bounds[1] > bounds[3]) fail("Límites XY inválidos");
    return query(index_path_, {bounds[0], bounds[1], -inf}, {bounds[2], bounds[3], inf}, limit,
                 [](const CloudPoint&) { return true; });
}

std::vector<CloudNeighbor> CloudStore::neighbors(std::span<const CloudXy> queries, double radius, bool upper) const {
    if (!std::isfinite(radius) || radius <= 0) fail("Radio de vecindad inválido");
    if (queries.empty()) return {};
    CloudXyz low{inf, inf, -inf}, high{-inf, -inf, inf};
    for (const auto& center : queries) {
        if (!finite(center)) fail("Consultas XY inválidas");
        for (unsigned axis = 0; axis < 2; ++axis) {
            low[axis] = std::min(low[axis], center[axis] - radius);
            high[axis] = std::max(high[axis], center[axis] + radius);
        }
    }
    constexpr double missing = std::numeric_limits<double>::quiet_NaN();
    std::vector<CloudNeighbor> result(queries.size(), {inf, {{missing, missing, missing}, -1}});
    visit_chunks(index_path_, low, high, [&](const Statement& stmt) {
        struct IndexedPoint {
            CloudPoint point;
            std::size_t row;
        };
        std::vector<IndexedPoint> chunk;
        decode(stmt, [&](const CloudPoint& point) { chunk.push_back({point, chunk.size()}); });
        const auto x = [](const IndexedPoint& entry) { return entry.point.xyz[0]; };
        // Restrict each exact query to points within its horizontal interval.
        std::ranges::stable_sort(chunk, {}, x);
        for (std::size_t i = 0; i < queries.size(); ++i) {
            const auto& center = queries[i];
            auto& best = result[i];
            std::optional<std::size_t> row;
            auto point = std::ranges::lower_bound(chunk, center[0] - radius, {}, x);
            for (; point != chunk.end() && x(*point) <= center[0] + radius; ++point) {
                const auto& candidate = point->point;
                const auto d2 = square(x(*point) - center[0]) + square(candidate.xyz[1] - center[1]);
                if (d2 > square(radius)) continue;
                const auto distance = std::sqrt(d2);
                const bool improves = upper && std::isfinite(best.distance) && candidate.xyz[2] != best.point.xyz[2]
                    ? candidate.xyz[2] > best.point.xyz[2]
                    : distance < best.distance || (distance == best.distance && row && point->row < *row);
                if (improves) {
                    best = {distance, candidate};
                    row = point->row;
                }
            }
        }
    });
    return result;
}

std::vector<CloudNeighbor> CloudStore::nearest_xy(std::span<const CloudXy> queries, double radius) const {
    return neighbors(queries, radius, false);
}

std::vector<CloudNeighbor> CloudStore::upper_xy(std::span<const CloudXy> queries, double radius) const {
    return neighbors(queries, radius, true);
}

} // namespace fusionforce
