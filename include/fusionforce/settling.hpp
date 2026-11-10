#pragma once

#include "fusionforce/placement.hpp"
#include "fusionforce/truck_step.hpp"

namespace fusionforce {

struct RestingPlacement {
    PlacementSeed pose;
    bool grounded = true;
    bool settled = false;
    double settling_time = 0;
};

// Resolve the geometric drop seed into the pose used at the start of playback.
RestingPlacement settle_vehicle(const PlacementSeed& seed, const CloudStore& cloud,
                                const TruckGeometry& truck, bool lightweight,
                                const TruckTerrain& terrain);

} // namespace fusionforce
