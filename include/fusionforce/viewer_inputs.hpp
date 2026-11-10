#pragma once

#include "fusionforce/simulation.hpp"

#include <nlohmann/json_fwd.hpp>

namespace fusionforce {

SimulationInput simulation_input(const nlohmann::json& request, CloudXyz center);
TruckTerrain truck_terrain(const nlohmann::json& request);
nlohmann::json place_from_request(const nlohmann::json& request, const CloudStore& cloud,
                                  const TruckGeometry& truck);

} // namespace fusionforce
