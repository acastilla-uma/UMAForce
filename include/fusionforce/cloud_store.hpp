#pragma once

#include <nlohmann/json_fwd.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace fusionforce {

using CloudXyz = std::array<double, 3>;
using CloudXy = std::array<double, 2>;
using CloudLabels = std::map<std::string, std::string>;

struct CloudPoint {
    CloudXyz xyz;
    std::int64_t label = -1;
};

struct CloudMetadata {
    std::filesystem::path path;
    std::string sha256;
    std::array<CloudXyz, 2> bounds;
    CloudXyz center;
    std::optional<std::string> label_property;
    CloudLabels label_mapping;
    std::uint64_t count = 0;
    std::uint64_t source_size = 0;
    std::int64_t source_mtime_ns = 0;
    std::uint64_t chunks = 0;
    double index_seconds = 0;
    static constexpr int version = 1;
    static constexpr bool complete = true;
    static constexpr std::string_view coordinate_frame = "PLY XYZ; metres; Z up assumed";
};

nlohmann::json cloud_metadata_json(const CloudMetadata& metadata);

struct CloudOptions {
    std::filesystem::path cache_dir = ".cache/clouds";
    std::optional<CloudLabels> label_mapping;
    std::function<void(std::string_view, std::uint64_t, std::uint64_t)> progress;
    std::function<bool()> cancel;
};

struct CloudNeighbor {
    double distance;
    CloudPoint point;
};

struct CloudOverview {
    std::vector<CloudPoint> points;
    std::uint64_t total = 0;
};

class CloudStore {
public:
    explicit CloudStore(std::filesystem::path index_path);
    static CloudStore open(const std::filesystem::path& path, const CloudOptions& options = {});

    const CloudMetadata& metadata() const noexcept { return metadata_; }
    const std::filesystem::path& index_path() const noexcept { return index_path_; }
    std::vector<CloudPoint> all_points() const;
    std::vector<CloudPoint> overview(std::size_t limit = 30000) const;
    std::vector<CloudPoint> sphere(CloudXyz center, double radius = 5,
                                   std::optional<std::size_t> limit = {}) const;
    CloudOverview sphere_overview(CloudXyz center, double radius = 5, std::size_t limit = 100000) const;
    std::vector<CloudPoint> region_xy(std::array<double, 4> bounds,
                                      std::optional<std::size_t> limit = {}) const;
    std::vector<CloudNeighbor> nearest_xy(std::span<const CloudXy> queries, double radius = 2) const;
    std::vector<CloudNeighbor> upper_xy(std::span<const CloudXy> queries, double radius = .35) const;
    void close() noexcept {}

private:
    std::vector<CloudNeighbor> neighbors(std::span<const CloudXy> queries, double radius, bool upper) const;
    std::filesystem::path index_path_;
    CloudMetadata metadata_;
};

}
