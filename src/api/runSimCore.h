#pragma once

#include "EigenSupport.h"
#include "configFileJSON.h"
#include "deformationModelManager.h"

#include <array>
#include <string>
#include <vector>

namespace pgo {
namespace api {

struct FixedVertexAttachment {
    std::string           filename;
    std::array<double, 3> movement{};
    double                coeff = 0.0;
};

struct ExternalObject {
    std::string           filename;
    std::array<double, 3> movement{};
};

struct RunSimConfig {
    std::string                                            tetMeshFilename;
    std::string                                            surfaceMeshFilename;
    std::vector<FixedVertexAttachment>                     fixedVertices;
    std::vector<ExternalObject>                            externalObjects;
    EigenSupport::V3d                                      extAcc;
    EigenSupport::V3d                                      initialVel;
    EigenSupport::V3d                                      initialDisp;
    double                                                 scale                = 1.0;
    double                                                 timestep             = 1.0;
    double                                                 contactStiffness     = 0.0;
    int                                                    contactSamples       = 1;
    double                                                 contactFrictionCoeff = 0.0;
    double                                                 contactVelEps        = 0.0;
    double                                                 solverEps            = 0.0;
    int                                                    solverMaxIter        = 0;
    std::array<double, 2>                                  dampingParams{};
    SolidDeformationModel::DeformationModelElasticMaterial elasticMaterial{};
    int                                                    numSimSteps  = 0;
    int                                                    dumpInterval = 1;
    std::string                                            simType;
    std::string                                            outputFolder;
    bool                                                   deterministicMode = false;
    bool                                                   restartEnabled = false;
    bool                                                   restartPause   = false;
};

struct RunSimState {
    bool              hasRestart = false;
    int               frameStart = 0;
    EigenSupport::VXd u;
    EigenSupport::VXd uvel;
    EigenSupport::VXd uacc;
};

RunSimConfig parseRunSimConfig(const ConfigFileJSON& jconfig, const std::string& configFilePath);

int runSimFromConfig(const RunSimConfig& config);

}  // namespace api
}  // namespace pgo
