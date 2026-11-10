#include "fusionforce/cloud_store.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>

namespace {

namespace fs = std::filesystem;
using namespace fusionforce;

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

template<class Operation>
void rejects(Operation operation, const char* message) {
    try {
        operation();
    } catch (const std::exception&) {
        return;
    }
    throw std::runtime_error(message);
}

struct Fixture {
    fs::path path = fs::temp_directory_path() / ("fusionforce-cloud-" + std::to_string(std::random_device{}()));
    Fixture() { fs::create_directory(path); }
    ~Fixture() {
        std::error_code ignored;
        fs::remove_all(path, ignored);
    }
};

void write_bits(std::ostream& stream, std::uint64_t bits, int size, bool big) {
    for (int i = 0; i < size; ++i) {
        stream.put(static_cast<char>((bits >> (8 * (big ? size - 1 - i : i))) & 255));
    }
}

void write_binary(const fs::path& path, bool big, int count) {
    std::ofstream out(path, std::ios::binary);
    out << "ply\nformat binary_" << (big ? "big" : "little") << "_endian 1.0\nelement vertex " << count
        << "\nproperty short classification\nproperty float z\nproperty double x\nproperty int y\nend_header\n";
    for (int i = 0; i < count; ++i) {
        write_bits(out, std::uint16_t(i % 5 - 2), 2, big);
        write_bits(out, std::bit_cast<std::uint32_t>(float(i)), 4, big);
        write_bits(out, std::bit_cast<std::uint64_t>(double(i % 200)), 8, big);
        write_bits(out, std::uint32_t(i / 200), 4, big);
    }
}

void test_binary(const Fixture& fixture, bool big, int count) {
    const auto source = fixture.path / (big ? "big.ply" : "little.ply");
    write_binary(source, big, count);
    CloudOptions options;
    options.cache_dir = fixture.path / "cache";
    std::uint64_t indexed = 0;
    options.progress = [&](std::string_view stage, std::uint64_t done, std::uint64_t total) {
        if (stage == "index") {
            check(done > indexed && done <= total, "Index progress not monotonic");
            indexed = done;
        }
    };
    const auto cloud = CloudStore::open(source, options);
    const auto expected = static_cast<std::uint64_t>(count);
    check(indexed == expected && cloud.metadata().count == expected, "Import did not cover the entire cloud");
    auto points = cloud.all_points();
    std::ranges::sort(points, {}, [](const CloudPoint& point) { return point.xyz[2]; });
    check(points.size() == expected, "Wrong complete cloud size");
    for (int i = 0; i < count; ++i) {
        check(points[i].xyz == CloudXyz{double(i % 200), double(i / 200), double(i)} &&
              points[i].label == i % 5 - 2, "Point and classification differ after import");
    }
    indexed = 0;
    const auto reopened = CloudStore::open(source, options);
    check(indexed == 0 && cloud.index_path() == reopened.index_path(), "Existing cache not reused");
    const auto sample = reopened.overview(2);
    check(sample.size() <= 2 && sample.front().xyz == CloudXyz{0, 0, 0}, "Overview sampling order");
    const auto cached_path = cloud.index_path();
    std::ofstream(cached_path, std::ios::binary | std::ios::trunc) << "broken index";
    const auto repaired = CloudStore::open(source, options);
    check(indexed == expected && repaired.all_points().size() == expected, "Broken cache not rebuilt");
}

void test_queries(const Fixture& fixture) {
    const auto source = fixture.path / "query.PLY";
    std::ofstream(source, std::ios::binary) <<
        "ply\r\nformat ascii 1.0\r\nelement vertex 6\r\nproperty int label\r\n"
        "property double z\r\nproperty float x\r\nproperty double y\r\nend_header\r\n"
        "10 0 0 0\r\n11 4 .25 0\r\n12 3 .125 0\r\n13 0 1 0\r\n14 0 11 0\r\n15 0 -10 0";
    std::ofstream(source.string() + ".labels.json") << R"({"10": "grava", "-1": "desconocido"})";
    CloudOptions options;
    options.cache_dir = fixture.path / "cache";
    const auto cloud = CloudStore::open(source, options);
    check(cloud.metadata().label_mapping.at("10") == "grava", "Sidecar mapping missing");
    check(cloud.metadata().bounds[0] == CloudXyz{-10, 0, 0} &&
          cloud.metadata().bounds[1] == CloudXyz{11, 0, 4}, "Incorrect coordinate bounds");
    check(cloud.region_xy({0, 0, 1, 0}).size() == 4, "XY bounds should include both endpoints");
    check(cloud.sphere({0, 0, 0}, 1).size() == 2, "Sphere should use all three coordinates");
    rejects([&] { cloud.region_xy({0, 0, 1, 0}, 3); }, "Query limit silently truncated");
    const std::array query{CloudXy{0, 0}, CloudXy{21, 0}, CloudXy{1.25, 0}};
    const auto nearest = cloud.nearest_xy(query, .25);
    const auto upper = cloud.upper_xy(query, .25);
    check(nearest[0].point.label == 10 && upper[0].point.label == 11, "Nearest and upper queries confused");
    check(upper[0].distance == .25 && nearest[2].point.label == 13, "Neighbor radius must be inclusive");
    check(std::isinf(upper[1].distance) && std::isnan(upper[1].point.xyz[0]) && upper[1].point.label == -1,
          "Missing support sentinel changed");
    const auto preview = cloud.sphere_overview({0, 0, 0}, 20, 2);
    check(preview.total == 6 && preview.points.size() == 2, "Wrong sphere preview count");
    // NumPy PCG64 seeded with zero retains indices 3 and 2 in this ordered set.
    check(preview.points[0].label == 11 && preview.points[1].label == 12, "Wrong deterministic preview sample");
}

void test_cancelled_import(const Fixture& fixture) {
    const auto source = fixture.path / "cancel.ply";
    write_binary(source, true, 3);
    bool cancel = false;
    CloudOptions options;
    options.cache_dir = fixture.path / "cancel-cache";
    options.cancel = [&] { return cancel; };
    options.progress = [&](std::string_view stage, auto, auto) { cancel = stage == "index"; };
    rejects([&] { CloudStore::open(source, options); }, "Cancelled import succeeded");
    check(fs::is_empty(options.cache_dir), "Cancelled import left a partial cache");
    check(fs::is_regular_file(source), "Import modified the source");
}

}

int main() {
    try {
        const Fixture fixture;
        test_binary(fixture, true, 3);
        test_binary(fixture, false, 100001);
        test_queries(fixture);
        test_cancelled_import(fixture);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
