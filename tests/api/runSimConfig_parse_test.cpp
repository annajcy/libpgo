#include "configFileJSON.h"
#include "runSimCore.h"
#include "pgoLogging.h"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <random>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace test {

namespace {

class ScopedTempDir {
public:
    ScopedTempDir() {
        std::random_device rd;
        path_ = std::filesystem::temp_directory_path() /
                std::filesystem::path("libpgo-runsim-config-test-" + std::to_string(rd()) + "-" +
                                      std::to_string(rd()));
        std::filesystem::create_directories(path_);
    }

    ~ScopedTempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }

    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

bool writeTextFile(const std::filesystem::path& filePath, const std::string& content) {
    std::filesystem::create_directories(filePath.parent_path());
    std::ofstream os(filePath);
    if (!os) {
        return false;
    }

    os << content;
    return static_cast<bool>(os);
}

std::string readTextFile(const std::filesystem::path& filePath) {
    std::ifstream is(filePath);
    if (!is) {
        throw std::runtime_error("Failed to open text file for reading.");
    }

    return std::string((std::istreambuf_iterator<char>(is)), std::istreambuf_iterator<char>());
}

bool writeFoldedCubicBoxSurface(const std::filesystem::path& srcFile, const std::filesystem::path& dstFile) {
    std::ifstream is(srcFile);
    if (!is) {
        return false;
    }

    std::filesystem::create_directories(dstFile.parent_path());
    std::ofstream os(dstFile);
    if (!os) {
        return false;
    }

    std::string line;
    while (std::getline(is, line)) {
        if (!line.starts_with("v ")) {
            os << line << '\n';
            continue;
        }

        std::istringstream iss(line);
        std::string        tag;
        double             x = 0.0;
        double             y = 0.0;
        double             z = 0.0;
        iss >> tag >> x >> y >> z;
        if (!iss) {
            return false;
        }

        // Fold the top layer down near the bottom face to create a stable self near-contact scene.
        if (y > 0.25) {
            y = -0.45;
        }

        os << "v " << x << ' ' << y << ' ' << z << '\n';
    }

    return static_cast<bool>(os);
}

nlohmann::json makeBaseConfig() {
    return {
        {"surface-mesh", "surface.obj"},
        {"fixed-vertices", nlohmann::json::array()},
        {"external-objects", nlohmann::json::array()},
        {"g", {0.0, 0.0, 0.0}},
        {"init-vel", {0.0, 0.0, 0.0}},
        {"init-disp", {0.0, 0.0, 0.0}},
        {"scale", 1.0},
        {"timestep", 0.01},
        {"num-timestep", 1},
        {"damping-params", {0.0, 0.0}},
        {"sim-type", "static"},
        {"contact-stiffness", 0.0},
        {"contact-sample", 1},
        {"contact-friction-coeff", 0.0},
        {"contact-vel-eps", 1e-8},
        {"solver-eps", 1e-8},
        {"solver-max-iter", 2},
        {"elastic-material", "stable-neo"},
        {"dump-interval", 1},
        {"output", "result.obj"},
    };
}

pgo::api::RunSimConfig parseConfigFile(const std::filesystem::path& configFile) {
    pgo::ConfigFileJSON jconfig;
    if (!jconfig.open(configFile.string().c_str())) {
        throw std::runtime_error("Failed to open config json file.");
    }

    return pgo::api::parseRunSimConfig(jconfig, configFile.string());
}

std::string normalizedAbsolutePath(const std::filesystem::path& input) {
    return std::filesystem::absolute(input).lexically_normal().string();
}

std::filesystem::path repoRootPath() {
    return std::filesystem::absolute(std::filesystem::path(__FILE__)).parent_path().parent_path().parent_path();
}

}  // namespace

TEST(RunSimConfigParseTest, TetMeshConfigSelectsTetInputType) {
    ScopedTempDir tempDir;

    const std::filesystem::path configDir  = tempDir.path() / "project" / "config";
    const std::filesystem::path configFile = configDir / "sim.json";

    nlohmann::json j = makeBaseConfig();
    j["tet-mesh"]   = "../assets/tet/simple.veg";

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    const pgo::api::RunSimConfig config = parseConfigFile(configFile);

    EXPECT_EQ(config.mesh.volumetricMeshType, pgo::api::VolumetricMeshInputType::TET);
    EXPECT_EQ(config.mesh.tetMeshFilename, normalizedAbsolutePath(configDir / "../assets/tet/simple.veg"));
    EXPECT_TRUE(config.mesh.cubicMeshFilename.empty());
    EXPECT_EQ(config.mesh.surfaceMeshFilename, normalizedAbsolutePath(configDir / "surface.obj"));
}

TEST(RunSimConfigParseTest, CubicMeshConfigSelectsCubicInputType) {
    ScopedTempDir tempDir;

    const std::filesystem::path configDir  = tempDir.path() / "project" / "config";
    const std::filesystem::path configFile = configDir / "sim.json";

    nlohmann::json j = makeBaseConfig();
    j["cubic-mesh"] = "../assets/cubic/simple.veg";

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    const pgo::api::RunSimConfig config = parseConfigFile(configFile);

    EXPECT_EQ(config.mesh.volumetricMeshType, pgo::api::VolumetricMeshInputType::CUBIC);
    EXPECT_TRUE(config.mesh.tetMeshFilename.empty());
    EXPECT_EQ(config.mesh.cubicMeshFilename, normalizedAbsolutePath(configDir / "../assets/cubic/simple.veg"));
    EXPECT_EQ(config.mesh.surfaceMeshFilename, normalizedAbsolutePath(configDir / "surface.obj"));
}

TEST(RunSimConfigParseTest, RejectsConfigWithBothTetAndCubicMeshKeys) {
    ScopedTempDir tempDir;

    const std::filesystem::path configFile = tempDir.path() / "sim.json";

    nlohmann::json j = makeBaseConfig();
    j["tet-mesh"]   = "tet.veg";
    j["cubic-mesh"] = "cubic.veg";

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    EXPECT_THROW((void)parseConfigFile(configFile), std::runtime_error);
}

TEST(RunSimConfigParseTest, RejectsConfigWithMissingVolumetricMeshKey) {
    ScopedTempDir tempDir;

    const std::filesystem::path configFile = tempDir.path() / "sim.json";

    nlohmann::json j = makeBaseConfig();

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    EXPECT_THROW((void)parseConfigFile(configFile), std::runtime_error);
}

TEST(RunSimConfigParseTest, RejectsVolumetricMeshTypeHintMismatch) {
    ScopedTempDir tempDir;

    const std::filesystem::path configFile = tempDir.path() / "sim.json";

    nlohmann::json j             = makeBaseConfig();
    j["cubic-mesh"]             = "cubic.veg";
    j["volumetric-mesh-type"]   = "tet";

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    EXPECT_THROW((void)parseConfigFile(configFile), std::runtime_error);
}

TEST(RunSimConfigParseTest, AcceptsVolumetricMeshTypeHintWhenConsistent) {
    ScopedTempDir tempDir;

    const std::filesystem::path configFile = tempDir.path() / "sim.json";

    nlohmann::json j             = makeBaseConfig();
    j["cubic-mesh"]             = "cubic.veg";
    j["volumetric-mesh-type"]   = "cubic";

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    const pgo::api::RunSimConfig config = parseConfigFile(configFile);
    EXPECT_EQ(config.mesh.volumetricMeshType, pgo::api::VolumetricMeshInputType::CUBIC);
}

TEST(RunSimConfigParseTest, DefaultsToPenaltyContactModelAndDefaultIpcParameters) {
    ScopedTempDir tempDir;

    const std::filesystem::path configFile = tempDir.path() / "sim.json";

    nlohmann::json j = makeBaseConfig();
    j["cubic-mesh"] = "cubic.veg";

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    const pgo::api::RunSimConfig config = parseConfigFile(configFile);

    EXPECT_EQ(config.contact.contactModel, "penalty");
    EXPECT_DOUBLE_EQ(config.contact.ipcDhat, 1e-3);
    EXPECT_DOUBLE_EQ(config.contact.ipcKappa, 1e4);
    EXPECT_DOUBLE_EQ(config.contact.ipcAlphaSafety, 0.9);
    EXPECT_TRUE(config.contact.ipcEnableFeasibleLineSearch);
}

TEST(RunSimConfigParseTest, ParsesIpcBarrierContactParameters) {
    ScopedTempDir tempDir;

    const std::filesystem::path configFile = tempDir.path() / "sim.json";

    nlohmann::json j        = makeBaseConfig();
    j["cubic-mesh"]         = "cubic.veg";
    j["contact-model"]      = "ipc-barrier";
    j["ipc-dhat"]           = 0.02;
    j["ipc-kappa"]          = 2.5e5;
    j["ipc-alpha-safety"]   = 0.8;
    j["ipc-enable-feasible-line-search"] = false;

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    const pgo::api::RunSimConfig config = parseConfigFile(configFile);

    EXPECT_EQ(config.contact.contactModel, "ipc-barrier");
    EXPECT_DOUBLE_EQ(config.contact.ipcDhat, 0.02);
    EXPECT_DOUBLE_EQ(config.contact.ipcKappa, 2.5e5);
    EXPECT_DOUBLE_EQ(config.contact.ipcAlphaSafety, 0.8);
    EXPECT_FALSE(config.contact.ipcEnableFeasibleLineSearch);
}

TEST(RunSimConfigParseTest, IpcBarrierFeasibleLineSearchDefaultsToEnabled) {
    ScopedTempDir tempDir;

    const std::filesystem::path configFile = tempDir.path() / "sim.json";

    nlohmann::json j      = makeBaseConfig();
    j["cubic-mesh"]       = "cubic.veg";
    j["contact-model"]    = "ipc-barrier";
    j["ipc-dhat"]         = 0.02;
    j["ipc-kappa"]        = 2.5e5;
    j["ipc-alpha-safety"] = 0.8;

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    const pgo::api::RunSimConfig config = parseConfigFile(configFile);

    EXPECT_TRUE(config.contact.ipcEnableFeasibleLineSearch);
}

TEST(RunSimConfigParseTest, RejectsUnsupportedContactModel) {
    ScopedTempDir tempDir;

    const std::filesystem::path configFile = tempDir.path() / "sim.json";

    nlohmann::json j        = makeBaseConfig();
    j["cubic-mesh"]         = "cubic.veg";
    j["contact-model"]      = "unsupported";

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    EXPECT_THROW((void)parseConfigFile(configFile), std::runtime_error);
}

TEST(RunSimConfigParseTest, RejectsNonPositiveIpcDhat) {
    ScopedTempDir tempDir;

    const std::filesystem::path configFile = tempDir.path() / "sim.json";

    nlohmann::json j        = makeBaseConfig();
    j["cubic-mesh"]         = "cubic.veg";
    j["contact-model"]      = "ipc-barrier";
    j["ipc-dhat"]           = 0.0;

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    EXPECT_THROW((void)parseConfigFile(configFile), std::runtime_error);
}

TEST(RunSimConfigParseTest, RejectsNonPositiveIpcKappa) {
    ScopedTempDir tempDir;

    const std::filesystem::path configFile = tempDir.path() / "sim.json";

    nlohmann::json j        = makeBaseConfig();
    j["cubic-mesh"]         = "cubic.veg";
    j["contact-model"]      = "ipc-barrier";
    j["ipc-kappa"]          = 0.0;

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    EXPECT_THROW((void)parseConfigFile(configFile), std::runtime_error);
}

TEST(RunSimConfigParseTest, RejectsOutOfRangeIpcAlphaSafety) {
    ScopedTempDir tempDir;

    const std::filesystem::path configFile = tempDir.path() / "sim-a.json";
    const std::filesystem::path configFile2 = tempDir.path() / "sim-b.json";

    nlohmann::json j        = makeBaseConfig();
    j["cubic-mesh"]         = "cubic.veg";
    j["contact-model"]      = "ipc-barrier";
    j["ipc-alpha-safety"]   = 0.0;

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    nlohmann::json j2      = j;
    j2["ipc-alpha-safety"] = 1.1;
    ASSERT_TRUE(writeTextFile(configFile2, j2.dump(2)));

    EXPECT_THROW((void)parseConfigFile(configFile), std::runtime_error);
    EXPECT_THROW((void)parseConfigFile(configFile2), std::runtime_error);
}

TEST(RunSimConfigParseTest, RejectsFrictionWithIpcBarrierContactModel) {
    ScopedTempDir tempDir;

    const std::filesystem::path configFile = tempDir.path() / "sim.json";

    nlohmann::json j                 = makeBaseConfig();
    j["cubic-mesh"]                  = "cubic.veg";
    j["contact-model"]               = "ipc-barrier";
    j["contact-friction-coeff"]      = 0.3;

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    EXPECT_THROW((void)parseConfigFile(configFile), std::runtime_error);
}

TEST(RunSimConfigParseTest, RunSimFromConfigCubicDynamicSmokeTest) {
    ScopedTempDir tempDir;
    const std::filesystem::path repoRoot = repoRootPath();
    const std::filesystem::path cubicMeshFile = repoRoot / "examples" / "cubic-box" / "cubic-box.veg";
    const std::filesystem::path surfaceMeshFile = repoRoot / "examples" / "cubic-box" / "cubic-box.obj";
    const std::filesystem::path outputDir = tempDir.path() / "output";

    ASSERT_TRUE(std::filesystem::exists(cubicMeshFile));
    ASSERT_TRUE(std::filesystem::exists(surfaceMeshFile));

    pgo::Logging::init();

    pgo::api::RunSimConfig config;
    config.mesh.cubicMeshFilename = cubicMeshFile.string();
    config.mesh.surfaceMeshFilename = surfaceMeshFile.string();
    config.mesh.volumetricMeshType = pgo::api::VolumetricMeshInputType::CUBIC;
    config.scene.extAcc = pgo::EigenSupport::V3d::Zero();
    config.scene.initialVel = pgo::EigenSupport::V3d::Zero();
    config.scene.initialDisp = pgo::EigenSupport::V3d::Zero();
    config.mesh.scale = 1.0;
    config.simulation.timestep = 0.01;
    config.contact.contactStiffness = 0.0;
    config.contact.contactSamples = 1;
    config.contact.contactFrictionCoeff = 0.0;
    config.contact.contactVelEps = 1e-8;
    config.solver.solverEps = 1e-8;
    config.solver.solverMaxIter = 4;
    config.solver.dampingParams = {0.0, 0.0};
    config.solver.elasticMaterial = pgo::SolidDeformationModel::DeformationModelElasticMaterial::STABLE_NEO;
    config.simulation.numSimSteps = 2;
    config.simulation.dumpInterval = 1;
    config.simulation.simType = "dynamic";
    config.runtime.outputFolder = outputDir.string();
    config.runtime.deterministicMode = true;

    EXPECT_EQ(pgo::api::runSimFromConfig(config), 0);
    EXPECT_TRUE(std::filesystem::exists(outputDir / "deform0001.u"));
    EXPECT_TRUE(std::filesystem::exists(outputDir / "ret0001.obj"));

    std::error_code ec;
    std::filesystem::remove("fv.obj", ec);
}

TEST(RunSimConfigParseTest, RunSimFromConfigCubicDynamicIpcSmokeTest) {
    ScopedTempDir tempDir;
    const std::filesystem::path repoRoot = repoRootPath();
    const std::filesystem::path cubicMeshFile = repoRoot / "examples" / "cubic-box" / "cubic-box.veg";
    const std::filesystem::path surfaceMeshFile = repoRoot / "examples" / "cubic-box" / "cubic-box.obj";
    const std::filesystem::path externalObjectFile = repoRoot / "examples" / "bottom.obj";
    const std::filesystem::path outputDir = tempDir.path() / "output";

    ASSERT_TRUE(std::filesystem::exists(cubicMeshFile));
    ASSERT_TRUE(std::filesystem::exists(surfaceMeshFile));
    ASSERT_TRUE(std::filesystem::exists(externalObjectFile));

    pgo::Logging::init();

    pgo::api::RunSimConfig config;
    config.mesh.cubicMeshFilename = cubicMeshFile.string();
    config.mesh.surfaceMeshFilename = surfaceMeshFile.string();
    config.mesh.volumetricMeshType = pgo::api::VolumetricMeshInputType::CUBIC;
    config.scene.externalObjects.push_back({externalObjectFile.string(), {0.0, 0.0, 0.0}});
    config.scene.extAcc = pgo::EigenSupport::V3d::Zero();
    config.scene.initialVel = pgo::EigenSupport::V3d::Zero();
    config.scene.initialDisp = pgo::EigenSupport::V3d::Zero();
    config.mesh.scale = 1.0;
    config.simulation.timestep = 0.01;
    config.contact.contactStiffness = 0.0;
    config.contact.contactSamples = 1;
    config.contact.enableSelfContact = false;
    config.contact.contactFrictionCoeff = 0.0;
    config.contact.contactVelEps = 1e-8;
    config.contact.contactModel = "ipc-barrier";
    config.contact.ipcDhat = 0.01;
    config.contact.ipcKappa = 1e5;
    config.contact.ipcAlphaSafety = 0.9;
    config.solver.solverEps = 1e-8;
    config.solver.solverMaxIter = 4;
    config.solver.dampingParams = {0.0, 0.0};
    config.solver.elasticMaterial = pgo::SolidDeformationModel::DeformationModelElasticMaterial::STABLE_NEO;
    config.simulation.numSimSteps = 2;
    config.simulation.dumpInterval = 1;
    config.simulation.simType = "dynamic";
    config.runtime.outputFolder = outputDir.string();
    config.runtime.deterministicMode = true;

    EXPECT_EQ(pgo::api::runSimFromConfig(config), 0);
    EXPECT_TRUE(std::filesystem::exists(outputDir / "deform0001.u"));
    EXPECT_TRUE(std::filesystem::exists(outputDir / "ret0001.obj"));

    std::error_code ec;
    std::filesystem::remove("fv.obj", ec);
}

TEST(RunSimConfigParseTest, RunSimFromConfigCubicDynamicIpcNearContactActivatesExternalBarrier) {
    ScopedTempDir tempDir;
    const std::filesystem::path repoRoot = repoRootPath();
    const std::filesystem::path cubicMeshFile = repoRoot / "examples" / "cubic-box" / "cubic-box.veg";
    const std::filesystem::path surfaceMeshFile = repoRoot / "examples" / "cubic-box" / "cubic-box.obj";
    const std::filesystem::path externalObjectFile = repoRoot / "examples" / "bottom.obj";
    const std::filesystem::path outputDir = tempDir.path() / "output";
    const std::filesystem::path logFile = tempDir.path() / "runsim-ipc.log";
    const std::string logFilename = logFile.string();

    ASSERT_TRUE(std::filesystem::exists(cubicMeshFile));
    ASSERT_TRUE(std::filesystem::exists(surfaceMeshFile));
    ASSERT_TRUE(std::filesystem::exists(externalObjectFile));

    pgo::Logging::init(logFilename.c_str());

    pgo::api::RunSimConfig config;
    config.mesh.cubicMeshFilename = cubicMeshFile.string();
    config.mesh.surfaceMeshFilename = surfaceMeshFile.string();
    config.mesh.volumetricMeshType = pgo::api::VolumetricMeshInputType::CUBIC;
    config.scene.externalObjects.push_back({externalObjectFile.string(), {0.0, 0.0, 0.0}});
    config.scene.extAcc = pgo::EigenSupport::V3d::Zero();
    config.scene.initialVel = pgo::EigenSupport::V3d::Zero();
    config.scene.initialDisp = pgo::EigenSupport::V3d(0.0, -0.2, 0.0);
    config.mesh.scale = 1.0;
    config.simulation.timestep = 0.01;
    config.contact.contactStiffness = 0.0;
    config.contact.contactSamples = 1;
    config.contact.enableSelfContact = false;
    config.contact.contactFrictionCoeff = 0.0;
    config.contact.contactVelEps = 1e-8;
    config.contact.contactModel = "ipc-barrier";
    config.contact.ipcDhat = 0.01;
    config.contact.ipcKappa = 1e5;
    config.contact.ipcAlphaSafety = 0.9;
    config.solver.solverEps = 1e-8;
    config.solver.solverMaxIter = 4;
    config.solver.dampingParams = {0.0, 0.0};
    config.solver.elasticMaterial = pgo::SolidDeformationModel::DeformationModelElasticMaterial::STABLE_NEO;
    config.simulation.numSimSteps = 2;
    config.simulation.dumpInterval = 1;
    config.simulation.simType = "dynamic";
    config.runtime.outputFolder = outputDir.string();
    config.runtime.deterministicMode = true;

    EXPECT_EQ(pgo::api::runSimFromConfig(config), 0);
    EXPECT_TRUE(std::filesystem::exists(outputDir / "deform0001.u"));
    EXPECT_TRUE(std::filesystem::exists(outputDir / "ret0001.obj"));
    ASSERT_TRUE(std::filesystem::exists(logFile));

    const std::string logContents = readTextFile(logFile);
    const std::regex activeSamplesPattern(R"(# external active samples: ([1-9][0-9]*))");

    EXPECT_TRUE(std::regex_search(logContents, activeSamplesPattern));
    EXPECT_EQ(logContents.find("IPC barrier energy not yet implemented for external contact."), std::string::npos);

    std::error_code ec;
    std::filesystem::remove("fv.obj", ec);
}

TEST(RunSimConfigParseTest, RunSimFromConfigCubicDynamicIpcSelfBarrierSmokeTest) {
    ScopedTempDir tempDir;
    const std::filesystem::path repoRoot = repoRootPath();
    const std::filesystem::path cubicMeshFile = repoRoot / "examples" / "cubic-box" / "cubic-box.veg";
    const std::filesystem::path sourceSurfaceMeshFile = repoRoot / "examples" / "cubic-box" / "cubic-box.obj";
    const std::filesystem::path surfaceMeshFile = tempDir.path() / "folded-cubic-box.obj";
    const std::filesystem::path outputDir = tempDir.path() / "output";
    const std::filesystem::path logFile = tempDir.path() / "runsim-self-ipc.log";
    const std::string logFilename = logFile.string();

    ASSERT_TRUE(std::filesystem::exists(cubicMeshFile));
    ASSERT_TRUE(std::filesystem::exists(sourceSurfaceMeshFile));
    ASSERT_TRUE(writeFoldedCubicBoxSurface(sourceSurfaceMeshFile, surfaceMeshFile));

    pgo::Logging::init(logFilename.c_str());

    pgo::api::RunSimConfig config;
    config.mesh.cubicMeshFilename = cubicMeshFile.string();
    config.mesh.surfaceMeshFilename = surfaceMeshFile.string();
    config.mesh.volumetricMeshType = pgo::api::VolumetricMeshInputType::CUBIC;
    config.scene.extAcc = pgo::EigenSupport::V3d::Zero();
    config.scene.initialVel = pgo::EigenSupport::V3d::Zero();
    config.scene.initialDisp = pgo::EigenSupport::V3d::Zero();
    config.mesh.scale = 1.0;
    config.simulation.timestep = 0.01;
    config.contact.contactStiffness = 0.0;
    config.contact.contactSamples = 1;
    config.contact.enableSelfContact = true;
    config.contact.contactFrictionCoeff = 0.0;
    config.contact.contactVelEps = 1e-8;
    config.contact.contactModel = "ipc-barrier";
    config.contact.ipcDhat = 0.1;
    config.contact.ipcKappa = 10.0;
    config.contact.ipcAlphaSafety = 0.9;
    config.contact.ipcEnableFeasibleLineSearch = true;
    config.solver.solverEps = 1e-8;
    config.solver.solverMaxIter = 6;
    config.solver.dampingParams = {0.0, 0.0};
    config.solver.elasticMaterial = pgo::SolidDeformationModel::DeformationModelElasticMaterial::STABLE_NEO;
    config.simulation.numSimSteps = 2;
    config.simulation.dumpInterval = 1;
    config.simulation.simType = "dynamic";
    config.runtime.outputFolder = outputDir.string();
    config.runtime.deterministicMode = true;

    EXPECT_EQ(pgo::api::runSimFromConfig(config), 0);
    EXPECT_TRUE(std::filesystem::exists(outputDir / "deform0001.u"));
    EXPECT_TRUE(std::filesystem::exists(outputDir / "ret0001.obj"));
    ASSERT_TRUE(std::filesystem::exists(logFile));

    const std::string logContents = readTextFile(logFile);
    const std::regex activePairsPattern(R"(# self active pairs: ([1-9][0-9]*))");

    EXPECT_TRUE(std::regex_search(logContents, activePairsPattern));
    EXPECT_EQ(logContents.find("IPC barrier energy not yet implemented for self contact."), std::string::npos);

    std::error_code ec;
    std::filesystem::remove("fv.obj", ec);
}

}  // namespace test
