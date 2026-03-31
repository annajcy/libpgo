#include "configFileJSON.h"
#include "runSimCore.h"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace test {

namespace {

class ScopedTempDir {
public:
    ScopedTempDir() {
        path_ = std::filesystem::temp_directory_path() / std::filesystem::path("libpgo-runsim-config-test");
        path_ += std::to_string(std::rand());
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

}  // namespace

TEST(RunSimConfigParseTest, TetMeshConfigSelectsTetInputType) {
    ScopedTempDir tempDir;

    const std::filesystem::path configDir  = tempDir.path() / "project" / "config";
    const std::filesystem::path configFile = configDir / "sim.json";

    nlohmann::json j = makeBaseConfig();
    j["tet-mesh"]   = "../assets/tet/simple.veg";

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    const pgo::api::RunSimConfig config = parseConfigFile(configFile);

    EXPECT_EQ(config.volumetricMeshType, pgo::api::VolumetricMeshInputType::TET);
    EXPECT_EQ(config.tetMeshFilename, normalizedAbsolutePath(configDir / "../assets/tet/simple.veg"));
    EXPECT_TRUE(config.cubicMeshFilename.empty());
    EXPECT_EQ(config.surfaceMeshFilename, normalizedAbsolutePath(configDir / "surface.obj"));
}

TEST(RunSimConfigParseTest, CubicMeshConfigSelectsCubicInputType) {
    ScopedTempDir tempDir;

    const std::filesystem::path configDir  = tempDir.path() / "project" / "config";
    const std::filesystem::path configFile = configDir / "sim.json";

    nlohmann::json j = makeBaseConfig();
    j["cubic-mesh"] = "../assets/cubic/simple.veg";

    ASSERT_TRUE(writeTextFile(configFile, j.dump(2)));

    const pgo::api::RunSimConfig config = parseConfigFile(configFile);

    EXPECT_EQ(config.volumetricMeshType, pgo::api::VolumetricMeshInputType::CUBIC);
    EXPECT_TRUE(config.tetMeshFilename.empty());
    EXPECT_EQ(config.cubicMeshFilename, normalizedAbsolutePath(configDir / "../assets/cubic/simple.veg"));
    EXPECT_EQ(config.surfaceMeshFilename, normalizedAbsolutePath(configDir / "surface.obj"));
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
    EXPECT_EQ(config.volumetricMeshType, pgo::api::VolumetricMeshInputType::CUBIC);
}

}  // namespace test
