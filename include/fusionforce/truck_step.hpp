#pragma once

#include "fusionforce/terrain_patch.hpp"
#include "fusionforce/truck_geometry.hpp"

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace fusionforce {

struct TruckTerrainZone {
    CloudXyz center;
    double radius;
    std::string terrain_class;
};

struct TruckContactMaterial {
    std::string terrain_class;
    rigid::Material material;
};

struct TruckTerrain {
    std::map<std::string, rigid::Material> classes{
        {"desconocido", {1e6f, .7f, .7f}},
        {"grava", {7e5f, .6f, .6f}},
        {"roca", {3e6f, .85f, .85f}},
        {"plantas", {2e5f, .45f, .45f}},
        {"barro", {1e5f, .25f, .25f}},
        {"tierra", {5e5f, .65f, .65f}},
    };
    std::vector<TruckTerrainZone> zones;

    TruckContactMaterial material_at(const CloudPoint& point, const CloudLabels& labels) const;
};

class TruckCoverageError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

class TruckInstabilityError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct TruckStepResult {
    rigid::StepResult physics;
    std::vector<CloudXyz> global_points;
    CloudXyz global_cog;
    std::vector<TruckContactMaterial> contact_materials;
    std::vector<rigid::Vec3> moments;
};

// The cloud is fixed for a simulation run. Start a new stepper for each origin or cloud.
class TruckStepper {
public:
    explicit TruckStepper(CloudXyz origin) : origin_(origin), patch_(origin) {}

    TruckStepResult step(const CloudStore& cloud, const TruckGeometry& truck,
                         const rigid::State& state, const rigid::EngineConfig& config,
                         float speed, float steering, bool lightweight,
                         const TruckTerrain& terrain);

private:
    CloudXyz origin_;
    TerrainPatch patch_;
};

} // namespace fusionforce
