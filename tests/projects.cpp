#include "fusionforce/projects.hpp"
#include "fusionforce/sha256.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string_view>

namespace {

namespace fs = std::filesystem;
using nlohmann::json;
using namespace fusionforce;

void check(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <class Function>
void rejects(Function operation, const char* message)
{
    try {
        operation();
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error(message);
}

class WorkingFiles {
public:
    WorkingFiles()
        : path(fs::current_path() /
               ("project-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())))
    {
        check(fs::create_directory(path), "No se pudo crear el directorio de prueba");
    }

    ~WorkingFiles()
    {
        std::error_code error;
        fs::remove(path / "original.ply", error);
        fs::remove(path / "results.csv", error);
        fs::remove(path, error);
    }

    fs::path path;
};

void write(const fs::path& path, std::string_view bytes)
{
    std::ofstream stream(path, std::ios::binary);
    stream.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    check(stream.good(), "No se pudo escribir el archivo de prueba");
}

} // namespace

int main()
{
    WorkingFiles working;
    const auto ply = working.path / "original.ply";
    const auto csv = working.path / "results.csv";
    const std::string source = "ply\nformat ascii 1.0\nelement vertex 1\n"
                               "property float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n";
    const std::string csv_bytes = "time,x,y,z\n0,1,2,3\n";
    write(ply, source);
    write(csv, csv_bytes);
    check(sha256("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
          "SHA-256 conocido incorrecto");

    const auto path_bytes = ply.u8string();
    auto cloud = json{{"path", std::string(path_bytes.begin(), path_bytes.end())}, {"sha256", sha256(source)},
                      {"label_mapping", {{"1", "grava"}, {"-2", "roca"}}}};
    auto project = json::parse(R"({
        "csv_available":true,"playback_index":0,"vehicle":"MAN_10t","pose":{"x":1,"y":2,"z":3},
        "applied_pose":{"position":[1,2,3],"orientation":[1,0,0,0]},
        "route":[{"x":1,"y":2,"z":3,"speed":1,"acceleration":0.5}],
        "terrain":{"classes":{"grava":{"friction":0.65,"stiffness":800000}},
                   "zones":[{"center":[1,2,3],"radius":2,"class":"grava"}]},
        "frames":[{"time":0,"position":[1,2,3],"orientation":[1,0,0,0],
                   "points":[[1,2,0]],"normal":[[0,0,100]],"contact":[1],
                   "stability":{"pltr":{"value":null,"reason":"Se necesita una muestra anterior"}}}],
        "trajectory":[{"time":0,"position":[1,2,3],"orientation":[1,0,0,0]}],
        "warnings":"","config":{"obsolete":true},"configuration":{}
    })");

    const auto archive = export_archive(project, cloud, csv);
    fs::remove(ply);
    fs::remove(csv);
    const auto loaded = read_archive(archive);
    auto expected = project;
    expected.erase("config");
    expected.erase("configuration");
    expected["cloud"] = cloud;
    expected["cloud"].erase("path");
    check(loaded.project == expected && loaded.cloud == source && loaded.csv == csv_bytes,
          "Proyecto restaurado difiere del original");

    auto damaged = archive;
    const auto local_data = 30 + std::string_view("project.json").size();
    damaged[local_data] ^= 0x40;
    rejects([&] { read_archive(damaged); }, "Miembro dañado aceptado");
    rejects([&] { read_archive("not a zip"); }, "Datos inválidos aceptados");
    rejects([&] { read_archive(std::string_view(archive).substr(0, archive.size() - 1)); },
            "Archivo truncado aceptado");

    damaged = archive;
    const auto central = damaged.find("PK\x01\x02");
    check(central != std::string::npos, "Falta el directorio ZIP");
    damaged[central + 27] = static_cast<char>(0x7f);
    rejects([&] { read_archive(damaged); }, "Miembro sobredimensionado aceptado");

    damaged = archive;
    damaged.replace(central + 46, 12, "../abcde.txt");
    rejects([&] { read_archive(damaged); }, "Nombre de miembro inseguro aceptado");

    auto bad = project;
    bad["pose"]["x"] = true;
    rejects([&] { validate_project(bad); }, "Coordenada booleana aceptada");
    bad = project;
    bad["frames"][0]["points"] = json::array({"incorrecto"});
    rejects([&] { validate_project(bad); }, "Contacto inválido aceptado");
    bad = project;
    bad["playback_index"] = -1;
    rejects([&] { validate_project(bad); }, "Índice de reproducción negativo aceptado");
    bad = project;
    bad["frames"][0]["time"] = std::numeric_limits<double>::infinity();
    rejects([&] { validate_project(bad); }, "Número infinito aceptado");

    write(ply, "modificado");
    rejects([&] { export_archive(project, cloud); }, "PLY modificado aceptado");
    write(ply, source);
    project["csv_available"] = false;
    check(!read_archive(export_archive(project, cloud, csv)).csv,
          "Se guardó el CSV anterior aunque estaba desactivado");

    cloud["label_mapping"]["01"] = "grava";
    const auto bad_mapping = export_archive(project, cloud);
    rejects([&] { read_archive(bad_mapping); }, "Tabla de clases no canónica aceptada");
}
