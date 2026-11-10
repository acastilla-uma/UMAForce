#pragma once

#include "fusionforce/cloud_store.hpp"

#include <nlohmann/json.hpp>

namespace fusionforce {

using CloudProgress = std::function<void(std::string_view, std::uint64_t, std::uint64_t)>;
using CloudCancel = std::function<bool()>;

// The HTTP caller serializes operations and keeps borrowed cloud references
// within the same operation.
class ViewerWorkspace {
public:
    explicit ViewerWorkspace(std::filesystem::path viewer_root);

    nlohmann::json info() const;
    nlohmann::json load_path(const std::filesystem::path& path, CloudProgress progress = {}, CloudCancel cancel = {});
    nlohmann::json load(std::string_view bytes);
    nlohmann::json view(const nlohmann::json& center) const;
    nlohmann::json import_project(std::string_view bytes);
    std::string export_project(const nlohmann::json& project) const;

    const CloudStore& cloud() const;
    const std::filesystem::path& root() const noexcept { return root_; }
    const std::filesystem::path& csv_path() const noexcept { return csv_path_; }
    void set_csv_path(std::filesystem::path path) noexcept { csv_path_ = std::move(path); }
    void invalidate_csv() noexcept { csv_path_.clear(); }

private:
    nlohmann::json info_for(const CloudStore* store, std::span<const CloudPoint> points) const;

    std::filesystem::path root_;
    std::optional<CloudStore> cloud_;
    std::vector<CloudPoint> points_;
    std::filesystem::path csv_path_;
};

} // namespace fusionforce
