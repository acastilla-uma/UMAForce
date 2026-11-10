#include "fusionforce/viewer_inputs.hpp"
#include "fusionforce/viewer_results.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <thread>

#ifdef FUSIONFORCE_GPROF
extern "C" void moncontrol(int);
#endif

using namespace fusionforce;
using nlohmann::json;
using namespace std::chrono_literals;

double monotonic_seconds()
{
    timespec value{};
    if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        throw std::runtime_error("Cannot read monotonic clock");
    }
    return value.tv_sec + 1e-9 * value.tv_nsec;
}

int main(int argc, char** argv)
{
#ifdef FUSIONFORCE_GPROF
    moncontrol(0);
#endif
    try {
        if (argc != 5 && argc != 6) {
            throw std::invalid_argument("Usage: profile_man10t VIEWER PLY ROUTES CACHE [ROUTE_ID]");
        }
        std::ifstream fixture(argv[3]);
        const auto inputs = json::parse(fixture);
        CloudOptions options;
        options.cache_dir = argv[4];
        const auto cloud = CloudStore::open(argv[2], options);
        const auto& routes = inputs.at("routes");
        if (cloud.metadata().sha256 != inputs.at("ply_sha256").get<std::string>() || routes.size() != 10) {
            throw std::runtime_error("Fixture hash or route count mismatch");
        }
        const auto root = std::filesystem::path(argv[1]) / "assets/data/vehicle/MAN";
        const auto evaluate = [&](const json& payload) {
            const auto input = simulation_input(payload, cloud.metadata().center);
            const auto truck = load_truck_geometry(root, payload.at("vehicle"));
            auto result = simulate_truck(cloud, truck, truck_terrain(payload), input);
            return viewer_result_json(result, truck.vehicle_id, input.lightweight, false);
        };
        if (argc == 5) {
            auto warmup = routes.at(0).at("payload");
            warmup["duration"] = 1;
            evaluate(warmup);
            std::this_thread::sleep_for(3s);
            const double idle_start = monotonic_seconds();
            std::this_thread::sleep_for(6s);
            std::cout << json{{"idle_start_s", idle_start}, {"idle_end_s", monotonic_seconds()},
                {"cloud", {{"count", cloud.metadata().count}, {"sha256", cloud.metadata().sha256},
                    {"index_seconds", cloud.metadata().index_seconds}}}}.dump() << std::endl;
        }
        bool matched = false;
        for (const auto& route : routes) {
            if (argc == 6 && route.at("id") != argv[5]) continue;
            matched = true;
            if (argc == 5) std::this_thread::sleep_for(2s);
            const double start = monotonic_seconds();
            const auto cpu_start = std::clock();
#ifdef FUSIONFORCE_GPROF
            moncontrol(1);
#endif
            const auto result = evaluate(route.at("payload"));
#ifdef FUSIONFORCE_GPROF
            moncontrol(0);
#endif
            const double cpu = double(std::clock() - cpu_start) / CLOCKS_PER_SEC;
            const double end = monotonic_seconds();
            std::vector<CloudXy> xy;
            double path = 0;
            for (const auto& frame : result.at("frames")) {
                const auto point = frame.at("position").get<CloudXyz>();
                if (!xy.empty()) path += std::hypot(point[0] - xy.back()[0], point[1] - xy.back()[1]);
                xy.push_back({point[0], point[1]});
            }
            if (xy.empty()) throw std::runtime_error("Simulation returned no frames");
            const double displacement = std::hypot(xy.front()[0] - xy.back()[0], xy.front()[1] - xy.back()[1]);
            const double time = result.at("frames").back().at("time");
            if (result.at("status") != "complete" || result.at("samples") != 1001 ||
                std::abs(time - 10) > 1e-8 || displacement < 2) {
                throw std::runtime_error("Route does not satisfy original acceptance criteria");
            }
            std::cout << json{{"id", route.at("id")}, {"wall_s", end - start}, {"cpu_s", cpu},
                {"start_s", start}, {"end_s", end}, {"status", result.at("status")},
                {"samples", result.at("samples")}, {"simulated_s", time}, {"path_m", path},
                {"displacement_m", displacement}, {"warnings", result.at("warnings")}, {"xy", xy}}
                .dump() << std::endl;
        }
        if (!matched) throw std::runtime_error("Route ID not found");
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
