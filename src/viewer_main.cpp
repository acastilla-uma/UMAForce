#include "fusionforce/projects.hpp"
#include "fusionforce/viewer_inputs.hpp"
#include "fusionforce/viewer_results.hpp"
#include "fusionforce/viewer_workspace.hpp"

#include <httplib.h>

#include <algorithm>
#include <atomic>
#include <charconv>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <random>
#include <thread>

namespace {

using namespace fusionforce;
using nlohmann::json;

void send_json(httplib::Response& response, const json& result, int status = 200)
{
    response.status = status;
    response.set_header("Cache-Control", "no-store");
    response.set_content(result.dump(), "application/json; charset=utf-8");
}

void error(httplib::Response& response, std::string_view message, int status = 400)
{
    send_json(response, {{"error", message}}, status);
}

std::string identifier()
{
    std::random_device random;
    constexpr std::string_view digits = "0123456789abcdef";
    std::string result;
    for (int i = 0; i < 32; ++i) {
        result += digits[random() & 15];
    }
    return result;
}

struct ReleaseOperation {
    std::atomic_bool& busy;
    ~ReleaseOperation() { busy.store(false); }
};

class Viewer {
public:
    explicit Viewer(std::filesystem::path root) : workspace_(std::move(root)) {}

    void preload(const std::filesystem::path& path) { workspace_.load_path(path); }

    void configure(httplib::Server& http)
    {
        using HandlerResponse = httplib::Server::HandlerResponse;
        http.set_payload_max_length(project_archive_limit);
        http.set_pre_routing_handler([this](const httplib::Request& req, httplib::Response& res) {
            const auto host = req.get_header_value("Host");
            const auto origin = req.get_header_value("Origin");
            const auto slash = origin.find("://");
            const auto authority_start = slash == std::string::npos ? origin.size() : slash + 3;
            const auto authority_end = origin.find_first_of("/?#", authority_start);
            const auto authority = origin.substr(authority_start, authority_end - authority_start);
            const auto suffix = ":" + std::to_string(port);
            if ((host != "localhost" + suffix && host != "127.0.0.1" + suffix) ||
                (!origin.empty() && authority != host)) {
                error(res, "Origen local no autorizado");
                return HandlerResponse::Handled;
            }
            if (req.get_header_value("Sec-Fetch-Site") == "cross-site") {
                error(res, "Petición de otro sitio rechazada");
                return HandlerResponse::Handled;
            }
            return HandlerResponse::Unhandled;
        });
        http.set_exception_handler([](const httplib::Request&, httplib::Response& res, std::exception_ptr ex) {
            try {
                std::rethrow_exception(ex);
            } catch (const std::exception& exception) {
                error(res, exception.what());
            }
        });
        http.Get("/api/info", [this](const auto&, auto& res) {
            exclusive(res, [&] { send_json(res, workspace_.info()); });
        });
        http.Get("/api/job", [this](const auto&, auto& res) {
            std::lock_guard lock(jobs_mutex_);
            send_json(res, index_job_);
        });
        http.Get("/api/simulation/job", [this](const auto& req, auto& res) {
            std::lock_guard lock(jobs_mutex_);
            if (simulation_job_.is_null() || simulation_job_.at("job_id") != req.get_param_value("job_id")) {
                error(res, "Simulación no encontrada", 404);
            } else {
                send_json(res, simulation_job_);
            }
        });
        http.Get("/api/export.csv", [this](const auto&, auto& res) {
            exclusive(res, [&] { serve_csv(res); });
        });
        http.Get(R"(/(?:index\.html|app\.js|styles\.css|assets/uma-logo\.png|assets/data/vehicle/[^?%]+)?)",
            [this](const auto& req, auto& res) {
                if (req.path.find("..") != std::string::npos) {
                    error(res, "Ruta desconocida", 404);
                    return;
                }
                const auto path = workspace_.root() / (req.path == "/" ? "index.html" : req.path.substr(1));
                auto contents = std::ifstream(path, std::ios::binary);
                if (!contents || !std::filesystem::is_regular_file(path)) {
                    error(res, "Ruta desconocida", 404);
                    return;
                }
                const auto extension = path.extension().string();
                const char* type = extension == ".html" ? "text/html; charset=utf-8"
                    : extension == ".js" ? "text/javascript; charset=utf-8"
                    : extension == ".css" ? "text/css; charset=utf-8"
                    : extension == ".png" ? "image/png" : "application/octet-stream";
                res.set_content(std::string(std::istreambuf_iterator<char>(contents), {}), type);
            });
        http.Post(R"(/api/.*)", [this](const auto& req, auto& res) { post(req, res); });
        http.set_error_handler([](const httplib::Request&, httplib::Response& res) {
            if (res.body.empty() && res.status == 404) {
                error(res, "Ruta desconocida", 404);
            }
        });
    }

    int port = 8765;

private:
    template <class Function>
    void exclusive(httplib::Response& response, Function operation)
    {
        if (busy_.exchange(true)) {
            error(response, "Operación en curso", 409);
            return;
        }
        ReleaseOperation release{busy_};
        operation();
    }

    template <class Function>
    void launch(httplib::Response& response, Function operation)
    {
        if (busy_.exchange(true)) {
            error(response, "Operación en curso", 409);
            return;
        }
        try {
            worker_ = std::jthread([this, operation = std::move(operation)] {
                ReleaseOperation release{busy_};
                operation();
            });
        } catch (...) {
            busy_.store(false);
            throw;
        }
    }

    TruckGeometry geometry(const json& request) const
    {
        return load_truck_geometry(workspace_.root() / "assets/data/vehicle/MAN",
                                   request.value("vehicle", std::string("MAN_10t")));
    }

    json evaluate(const json& request, const std::function<void(SimulationProgress)>& progress = {})
    {
        const auto& cloud = workspace_.cloud();
        if (request.contains("disable_csv") && !request.at("disable_csv").is_boolean()) {
            throw std::invalid_argument("La opción de registro CSV debe ser booleana");
        }
        if (request.contains("lightweight_physics") && !request.at("lightweight_physics").is_boolean()) {
            throw std::invalid_argument("La opción de física ligera debe ser booleana");
        }
        workspace_.invalidate_csv();
        const auto input = simulation_input(request, cloud.metadata().center);
        const auto truck = geometry(request);
        const auto terrain = truck_terrain(request);
        const bool record_csv = !request.value("disable_csv", false);
        if (!record_csv) {
            return viewer_result_json(simulate_truck(cloud, truck, terrain, input, progress),
                                      truck.vehicle_id, input.lightweight, false);
        }
        const auto directory = workspace_.root() / ".cache/exports";
        std::filesystem::create_directories(directory);
        const auto id = identifier();
        const auto temporary = directory / (id + ".tmp");
        const auto final = directory / (id + ".csv");
        try {
            std::ofstream csv(temporary, std::ios::binary);
            csv.exceptions(std::ios::badbit | std::ios::failbit);
            auto result = simulate_truck(cloud, truck, terrain, input, progress, &csv);
            csv.close();
            auto payload = viewer_result_json(result, truck.vehicle_id, input.lightweight, true);
            std::filesystem::rename(temporary, final);
            workspace_.set_csv_path(final);
            return payload;
        } catch (...) {
            std::error_code ignored;
            std::filesystem::remove(temporary, ignored);
            throw;
        }
    }

    void serve_csv(httplib::Response& response) const
    {
        const auto path = workspace_.csv_path();
        auto stream = std::make_shared<std::ifstream>(path, std::ios::binary);
        if (!*stream) {
            throw std::invalid_argument("No hay un CSV disponible. Activa el registro CSV y vuelve a calcular.");
        }
        const auto size = std::filesystem::file_size(path);
        response.set_header("Content-Disposition", "attachment; filename=\"fusionforce.csv\"");
        response.set_content_provider(size, "text/csv; charset=utf-8",
            [stream](std::size_t offset, std::size_t length, httplib::DataSink& sink) {
                std::array<char, 65536> buffer;
                const auto count = std::min(length, buffer.size());
                stream->seekg(static_cast<std::streamoff>(offset));
                stream->read(buffer.data(), static_cast<std::streamsize>(count));
                return *stream && sink.write(buffer.data(), count);
            });
    }

    void start_index(const json& data, httplib::Response& res)
    {
        const auto path = data.value("path", std::string{});
        if (path.find_first_not_of(" \t\r\n") == std::string::npos) {
            throw std::invalid_argument("Introduce la ruta de una nube PLY");
        }
        std::lock_guard lock(jobs_mutex_);
        launch(res, [this, path] {
            try {
                {
                    std::lock_guard ready(jobs_mutex_);
                }
                auto info = workspace_.load_path(path, [this](auto stage, auto processed, auto total) {
                    std::lock_guard lock(jobs_mutex_);
                    index_job_["progress"] = {{"stage", stage}, {"processed", processed}, {"total", total}};
                }, [this] { return cancel_.load(); });
                std::lock_guard lock(jobs_mutex_);
                index_job_.update(json{{"status", "complete"}, {"info", std::move(info)}});
            } catch (const std::exception& exception) {
                std::lock_guard lock(jobs_mutex_);
                index_job_.update(json{{"status", cancel_.load() ? "cancelled" : "error"}, {"error", exception.what()}});
            }
        });
        if (res.status != 409) {
            cancel_.store(false);
            index_job_ = {{"status", "running"}, {"progress", 0}};
            send_json(res, {{"status", "running"}});
        }
    }

    void start_simulation(const json& data, httplib::Response& res)
    {
        if (!data.is_object()) {
            throw std::invalid_argument("Parámetros de simulación inválidos");
        }
        const auto id = identifier();
        std::lock_guard lock(jobs_mutex_);
        launch(res, [this, data] {
            try {
                auto result = evaluate(data, [this, &data](SimulationProgress value) {
                    auto progress = viewer_progress_json(value, value.dt);
                    if (value.stage == "complete") {
                        progress["stage"] = "finalizing";
                    }
                    progress["fraction"] = std::clamp(value.fraction, 0.0, .999);
                    std::lock_guard lock(jobs_mutex_);
                    simulation_job_["progress"] = std::move(progress);
                });
                std::lock_guard lock(jobs_mutex_);
                simulation_job_["progress"].update(json{{"stage", "complete"}, {"fraction", 1.0}});
                simulation_job_.update(json{{"status", "complete"}, {"result", std::move(result)}});
            } catch (const std::exception& exception) {
                std::lock_guard lock(jobs_mutex_);
                simulation_job_.update(json{{"status", "error"}, {"error", exception.what()}});
            }
        });
        if (res.status != 409) {
            simulation_job_ = {{"job_id", id}, {"status", "running"},
                {"progress", {{"stage", "preparation"}, {"processed", 0}, {"total", 0}, {"fraction", 0.0}}}};
            send_json(res, {{"job_id", id}, {"status", "running"}});
        }
    }

    void post(const httplib::Request& req, httplib::Response& res)
    {
        const std::size_t maximum = req.path == "/api/load" ? 100000000
            : req.path == "/api/project/import" ? project_archive_limit
            : req.path == "/api/project/export" ? project_document_limit : 8000000;
        if (req.body.empty() || req.body.size() > maximum) {
            throw std::invalid_argument("Tamaño de petición inválido; el archivo PLY debe ocupar entre 1 byte y 100 MB");
        }
        const bool binary = req.path == "/api/load" || req.path == "/api/project/import";
        const json data = binary ? json{} : json::parse(req.body);
        if (req.path == "/api/index") {
            start_index(data, res);
        } else if (req.path == "/api/simulation/start") {
            start_simulation(data, res);
        } else if (req.path == "/api/cancel") {
            cancel_.store(true);
            send_json(res, {{"status", "cancelling"}});
        } else {
            exclusive(res, [&] {
                if (req.path == "/api/load") {
                    send_json(res, workspace_.load(req.body));
                } else if (req.path == "/api/view") {
                    send_json(res, workspace_.view(data.at("center")));
                } else if (req.path == "/api/place") {
                    auto result = place_from_request(data, workspace_.cloud(), geometry(data));
                    workspace_.invalidate_csv();
                    send_json(res, result);
                } else if (req.path == "/api/evaluate") {
                    send_json(res, evaluate(data));
                } else if (req.path == "/api/project/import") {
                    send_json(res, workspace_.import_project(req.body));
                } else if (req.path == "/api/project/export") {
                    res.set_header("Content-Disposition", "attachment; filename=\"fusionforce.fusionforce\"");
                    res.set_header("Cache-Control", "no-store");
                    res.set_content(workspace_.export_project(data.at("project")), "application/zip");
                } else {
                    error(res, "Ruta desconocida");
                }
            });
        }
    }

    ViewerWorkspace workspace_;
    std::atomic_bool busy_{false};
    std::atomic_bool cancel_{false};
    std::mutex jobs_mutex_;
    json index_job_ = {{"status", "idle"}};
    json simulation_job_;
    std::jthread worker_;
};

} // namespace

int main(int argc, char* argv[])
{
    try {
        std::filesystem::path root = FUSIONFORCE_VIEWER_ROOT;
        std::filesystem::path ply;
        int port = 8765;
        for (int i = 1; i < argc; ++i) {
            const std::string_view option(argv[i]);
            if (i + 1 >= argc) {
                throw std::invalid_argument("Uso: fusionforce-viewer [--root carpeta] [--ply nube.ply] [--port puerto]");
            }
            const std::string_view value(argv[++i]);
            if (option == "--root") {
                root = value;
            } else if (option == "--ply") {
                ply = value;
            } else if (option == "--port") {
                const auto end = value.data() + value.size();
                const auto parsed = std::from_chars(value.data(), end, port);
                if (parsed.ec != std::errc{} || parsed.ptr != end || port < 0 || port > 65535) {
                    throw std::invalid_argument("Puerto inválido");
                }
            } else {
                throw std::invalid_argument("Opción desconocida: " + std::string(option));
            }
        }
        Viewer viewer(root);
        httplib::Server server;
        viewer.configure(server);
        if (!ply.empty()) {
            viewer.preload(ply);
        }
        viewer.port = port == 0 ? server.bind_to_any_port("127.0.0.1")
                                : server.bind_to_port("127.0.0.1", port) ? port : -1;
        if (viewer.port <= 0) {
            throw std::runtime_error("No se pudo abrir el puerto local");
        }
        std::cout << "Abre http://127.0.0.1:" << viewer.port << std::endl;
        return server.listen_after_bind() ? 0 : 1;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
