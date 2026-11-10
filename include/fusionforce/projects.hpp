#pragma once

#include <nlohmann/json.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace fusionforce {

inline constexpr std::size_t project_cloud_limit = 100'000'000;
inline constexpr std::size_t project_document_limit = 64'000'000;
inline constexpr std::size_t project_csv_limit = 128'000'000;
inline constexpr std::size_t project_archive_limit =
    project_cloud_limit + project_document_limit + project_csv_limit + 1'000'000;

struct PortableProject {
    nlohmann::json project;
    std::string cloud;
    std::optional<std::string> csv;
};

void validate_project(const nlohmann::json& project);
std::string export_archive(nlohmann::json project, nlohmann::json cloud,
                           const std::filesystem::path& csv_path = {});
PortableProject read_archive(std::string_view raw);

} // namespace fusionforce
