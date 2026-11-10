#include "fusionforce/viewer_workspace.hpp"
#include "fusionforce/projects.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <random>
#include <stdexcept>

namespace fusionforce {
namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;

Json read_json(const fs::path& path)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::runtime_error("No se pudo leer " + path.filename().string());
    return Json::parse(stream);
}

void write_file(const fs::path& path, std::string_view data)
{
    std::ofstream stream(path, std::ios::binary);
    stream.write(data.data(), static_cast<std::streamsize>(data.size()));
    stream.close();
    if (!stream) throw std::runtime_error("No se pudo escribir " + path.filename().string());
}

Json point_payload(std::span<const CloudPoint> points)
{
    Json::array_t xyz, labels;
    xyz.reserve(points.size());
    labels.reserve(points.size());
    for (const auto& point : points) {
        xyz.emplace_back(point.xyz);
        labels.emplace_back(point.label);
    }
    return {{"points", std::move(xyz)}, {"labels", std::move(labels)}};
}

Json catalog(const fs::path& root)
{
    auto result = Json::array();
    for (std::string model : {"MAN_5t", "MAN_7t", "MAN_10t"}) {
        const auto body = read_json(root / "assets/data/vehicle/MAN/chassis" / (model + "_Chassis.json"));
        auto name = model;
        std::ranges::replace(name, '_', ' ');
        result.push_back({{"id", model}, {"name", name + " · rígido experimental"},
                          {"mass", body.at("Components").at(0).at("Mass")},
                          {"family", "ruedas"}, {"status", "experimental"},
                          {"limitations", "Masa e inercia del chasis; suspensión fija, sin motor, transmisión ni neumáticos deformables. No validado como vehículo completo."},
                          {"mesh", "/assets/data/vehicle/MAN/meshes/" + model + "_chassis.obj"}});
    }
    return result;
}

Json terrain_defaults()
{
    return {{"desconocido", {{"friction", .7}, {"stiffness", 1e6}}},
            {"grava", {{"friction", .6}, {"stiffness", 7e5}}},
            {"roca", {{"friction", .85}, {"stiffness", 3e6}}},
            {"plantas", {{"friction", .45}, {"stiffness", 2e5}}},
            {"barro", {{"friction", .25}, {"stiffness", 1e5}}},
            {"tierra", {{"friction", .65}, {"stiffness", 5e5}}}};
}

// Each import owns a fresh directory. Remove only known files on rollback.
class StagedCloud {
public:
    StagedCloud(const fs::path& parent, std::string_view prefix)
    {
        fs::create_directories(parent);
        std::random_device random;
        for (int attempt = 0; attempt < 10; ++attempt) {
            auto candidate = parent / (std::string(prefix) + std::to_string(random()) + "-" + std::to_string(random()));
            if (fs::create_directory(candidate)) {
                directory_ = std::move(candidate);
                return;
            }
        }
        throw std::runtime_error("No se pudo preparar la importación de la nube");
    }
    StagedCloud(const StagedCloud&) = delete;
    StagedCloud& operator=(const StagedCloud&) = delete;
    ~StagedCloud()
    {
        if (keep_) return;
        std::error_code ignored;
        for (const char* name : {"cloud.ply", "cloud.ply.labels.json", "results.csv"}) {
            fs::remove(directory_ / name, ignored);
        }
        fs::remove(directory_, ignored);
    }
    fs::path file(const char* name) const { return directory_ / name; }
    void keep() noexcept { keep_ = true; }

private:
    fs::path directory_;
    bool keep_ = false;
};

} // namespace

ViewerWorkspace::ViewerWorkspace(fs::path viewer_root) : root_(fs::absolute(std::move(viewer_root))) {}

Json ViewerWorkspace::info_for(const CloudStore* store, std::span<const CloudPoint> points) const
{
    auto result = point_payload(points);
    result["center"] = store ? store->metadata().center : CloudXyz{};
    result["cloud"] = store ? cloud_metadata_json(store->metadata()) : Json(nullptr);
    auto vehicles = catalog(root_);
    result["model"] = vehicles.back();
    result["catalog"] = std::move(vehicles);
    const auto audit = root_ / "catalog.json";
    result["catalog_audit"] = fs::exists(audit) ? read_json(audit) : Json::object();
    result["terrain_defaults"] = terrain_defaults();
    return result;
}

Json ViewerWorkspace::info() const
{
    return info_for(cloud_ ? &*cloud_ : nullptr, points_);
}

Json ViewerWorkspace::load_path(const fs::path& path, CloudProgress progress, CloudCancel cancel)
{
    CloudOptions options;
    options.cache_dir = root_ / ".cache" / "clouds";
    options.progress = std::move(progress);
    options.cancel = std::move(cancel);
    auto candidate = CloudStore::open(path, options);
    auto points = candidate.all_points();
    auto result = info_for(&candidate, points);
    if (options.cancel && options.cancel()) {
        throw std::runtime_error("Indexación cancelada; el archivo original no se ha modificado");
    }
    cloud_ = std::move(candidate);
    points_ = std::move(points);
    invalidate_csv();
    return result;
}

Json ViewerWorkspace::load(std::string_view bytes)
{
    StagedCloud staged(root_ / ".cache" / "uploads", "upload-");
    const auto source = staged.file("cloud.ply");
    write_file(source, bytes);
    auto result = load_path(source);
    staged.keep();
    return result;
}

const CloudStore& ViewerWorkspace::cloud() const
{
    if (!cloud_) throw std::runtime_error("Carga primero una nube PLY");
    return *cloud_;
}

Json ViewerWorkspace::view(const Json& center) const
{
    const auto& store = cloud();
    if (!center.is_array() || center.size() != 3 ||
        !std::ranges::all_of(center, [](const Json& value) {
            return value.is_number() && std::isfinite(value.get<double>());
        })) {
        throw std::runtime_error("La vista local requiere un centro XYZ y radio de 5 m");
    }
    const auto overview = store.sphere_overview(center.get<CloudXyz>());
    auto result = point_payload(overview.points);
    result["radius"] = 5;
    result["visual_only"] = true;
    result["total"] = overview.total;
    result["sampled"] = overview.total > overview.points.size();
    return result;
}

std::string ViewerWorkspace::export_project(const Json& project) const
{
    if (!cloud_) throw std::runtime_error("Carga primero una nube");
    return export_archive(project, cloud_metadata_json(cloud_->metadata()), csv_path_);
}

Json ViewerWorkspace::import_project(std::string_view bytes)
{
    auto archived = read_archive(bytes);
    const auto& reference = archived.project.at("cloud");
    const auto mapping = reference.value("label_mapping", Json::object());
    StagedCloud staged(root_ / ".cache" / "projects", "project-");
    const auto source = staged.file("cloud.ply");
    write_file(source, archived.cloud);
    write_file(staged.file("cloud.ply.labels.json"), mapping.dump());
    fs::path csv;
    if (archived.csv) {
        csv = staged.file("results.csv");
        write_file(csv, *archived.csv);
    }
    CloudOptions options;
    options.cache_dir = root_ / ".cache" / "clouds";
    auto candidate = CloudStore::open(source, options);
    if (candidate.metadata().sha256 != reference.at("sha256").get_ref<const std::string&>()) {
        throw std::runtime_error("La huella de la nube no coincide con el proyecto");
    }
    if (Json(candidate.metadata().label_mapping) != mapping) {
        throw std::runtime_error("La tabla de clases de la nube cambió desde el guardado");
    }
    auto points = candidate.all_points();
    Json result = {{"project", std::move(archived.project)}, {"info", info_for(&candidate, points)},
                   {"csv_available", archived.csv.has_value()}};
    cloud_ = std::move(candidate);
    points_ = std::move(points);
    set_csv_path(std::move(csv));
    staged.keep();
    return result;
}

} // namespace fusionforce
