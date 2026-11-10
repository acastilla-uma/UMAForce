#pragma once

#include "fusionforce/simulation.hpp"

#include <nlohmann/json.hpp>
#include <string_view>

namespace fusionforce {

nlohmann::json viewer_frame_json(const SimulationFrame& frame, bool lightweight);
nlohmann::json viewer_result_json(const SimulationResult& result, std::string_view vehicle,
                                  bool lightweight, bool record_csv);
nlohmann::json viewer_progress_json(const SimulationProgress& progress, double dt);

} // namespace fusionforce
