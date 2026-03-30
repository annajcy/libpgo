#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#ifndef PGO_TEST_RUN_SIM_BIN
#define PGO_TEST_RUN_SIM_BIN ""
#endif

namespace test {

namespace {
std::string quotePath(const std::filesystem::path& p) {
    return "\"" + p.string() + "\"";
}

std::filesystem::path getRunSimBinaryPath() {
    const char* bin = std::getenv("RUN_SIM_BIN");
    if (bin != nullptr && !std::string(bin).empty()) {
        return std::filesystem::path(bin);
    }

    if (std::string(PGO_TEST_RUN_SIM_BIN).empty()) {
        return {};
    }

    return std::filesystem::path(PGO_TEST_RUN_SIM_BIN);
}

int runCommand(const std::string& command) {
    return std::system(command.c_str());
}

bool writeTextFile(const std::filesystem::path& filePath, const std::string& content) {
    std::filesystem::create_directories(filePath.parent_path());
    std::ofstream os(filePath);
    if (!os) {
        return false;
    }
    os << content;
    return static_cast<bool>(os);
}

std::vector<std::filesystem::path> listRelativeFiles(const std::filesystem::path& root) {
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
        if (entry.is_regular_file()) {
            files.push_back(std::filesystem::relative(entry.path(), root));
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

std::string readFileBytes(const std::filesystem::path& filePath) {
    std::ifstream is(filePath, std::ios::binary);
    if (!is) {
        return {};
    }

    return std::string((std::istreambuf_iterator<char>(is)), std::istreambuf_iterator<char>());
}

void expectDirectoriesEqual(const std::filesystem::path& lhs, const std::filesystem::path& rhs) {
    const std::vector<std::filesystem::path> lhsFiles = listRelativeFiles(lhs);
    const std::vector<std::filesystem::path> rhsFiles = listRelativeFiles(rhs);

    ASSERT_EQ(lhsFiles, rhsFiles);
    for (const auto& relativeFile : lhsFiles) {
        EXPECT_EQ(readFileBytes(lhs / relativeFile), readFileBytes(rhs / relativeFile)) << relativeFile.string();
    }
}

class ScopedTempDir {
public:
    ScopedTempDir() {
        path_ = std::filesystem::temp_directory_path() / std::filesystem::path("libpgo-runSim-test");
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
}  // namespace

TEST(RunSimToolTest, ResolvesAssetAndOutputPathsRelativeToConfigDirectory) {
    const std::filesystem::path runSimBin = getRunSimBinaryPath();
    ASSERT_FALSE(runSimBin.empty());
    ASSERT_TRUE(std::filesystem::exists(runSimBin));

    ScopedTempDir tempDir;

    const std::filesystem::path projectRoot  = tempDir.path() / "project";
    const std::filesystem::path runnerDir    = tempDir.path() / "runner";
    const std::filesystem::path configDir    = projectRoot / "config";
    const std::filesystem::path tetMeshFile  = projectRoot / "assets" / "tet" / "simple.veg";
    const std::filesystem::path surfaceFile  = projectRoot / "assets" / "surface" / "simple.obj";
    const std::filesystem::path fixedFile    = projectRoot / "assets" / "fixed" / "fixed.txt";
    const std::filesystem::path configFile   = configDir / "sim.json";
    const std::filesystem::path outputFile   = configDir / "result.obj";
    const std::filesystem::path runnerOutput = runnerDir / "result.obj";

    ASSERT_TRUE(writeTextFile(tetMeshFile, R"(# Minimal tet mesh
*VERTICES
4 3 0 0
1 0 0 0
2 1 0 0
3 0 1 0
4 0 0 1

*ELEMENTS
TET
1 4 0
1 1 2 3 4
)")
    );

    ASSERT_TRUE(writeTextFile(surfaceFile, R"(v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
f 1 3 2
f 1 2 4
f 2 3 4
f 3 1 4
)")
    );

    ASSERT_TRUE(writeTextFile(fixedFile, "0\n1\n2\n3\n"));

    ASSERT_TRUE(writeTextFile(configFile, R"({
  "tet-mesh": "../assets/tet/simple.veg",
  "surface-mesh": "../assets/surface/simple.obj",
  "fixed-vertices": [
    {
      "filename": "../assets/fixed/fixed.txt",
      "movement": [0, 0, 0],
      "coeff": 1.0
    }
  ],
  "external-objects": [],
  "g": [0, 0, 0],
  "init-vel": [0, 0, 0],
  "init-disp": [0, 0, 0],
  "scale": 1.0,
  "timestep": 0.01,
  "num-timestep": 1,
  "damping-params": [0, 0],
  "sim-type": "static",
  "contact-stiffness": 0.0,
  "contact-sample": 1,
  "contact-friction-coeff": 0.0,
  "contact-vel-eps": 1e-8,
  "solver-eps": 1e-8,
  "solver-max-iter": 2,
  "elastic-material": "stable-neo",
  "dump-interval": 1,
  "output": "result.obj"
})")
    );

    std::filesystem::create_directories(runnerDir);

    std::ostringstream cmd;
    cmd << "cd " << quotePath(runnerDir) << " && " << quotePath(runSimBin) << " "
        << quotePath(std::filesystem::path("../project/config/sim.json"));

    ASSERT_EQ(runCommand(cmd.str()), 0);
    EXPECT_TRUE(std::filesystem::exists(outputFile));
    EXPECT_FALSE(std::filesystem::exists(runnerOutput));
}

TEST(RunSimToolTest, DeterministicFlagEnablesSingleThreadedMode) {
    const std::filesystem::path runSimBin = getRunSimBinaryPath();
    ASSERT_FALSE(runSimBin.empty());
    ASSERT_TRUE(std::filesystem::exists(runSimBin));

    ScopedTempDir tempDir;

    const std::filesystem::path projectRoot = tempDir.path() / "project";
    const std::filesystem::path runnerDir   = tempDir.path() / "runner";
    const std::filesystem::path configDir   = projectRoot / "config";
    const std::filesystem::path tetMeshFile = projectRoot / "assets" / "tet" / "simple.veg";
    const std::filesystem::path surfaceFile = projectRoot / "assets" / "surface" / "simple.obj";
    const std::filesystem::path fixedFile   = projectRoot / "assets" / "fixed" / "fixed.txt";
    const std::filesystem::path configFile  = configDir / "sim.json";
    const std::filesystem::path outputFile  = configDir / "result.obj";
    const std::filesystem::path logFile     = runnerDir / "run.log";

    ASSERT_TRUE(writeTextFile(tetMeshFile, R"(# Minimal tet mesh
*VERTICES
4 3 0 0
1 0 0 0
2 1 0 0
3 0 1 0
4 0 0 1

*ELEMENTS
TET
1 4 0
1 1 2 3 4
)")
    );

    ASSERT_TRUE(writeTextFile(surfaceFile, R"(v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
f 1 3 2
f 1 2 4
f 2 3 4
f 3 1 4
)")
    );

    ASSERT_TRUE(writeTextFile(fixedFile, "0\n1\n2\n3\n"));

    ASSERT_TRUE(writeTextFile(configFile, R"({
  "tet-mesh": "../assets/tet/simple.veg",
  "surface-mesh": "../assets/surface/simple.obj",
  "fixed-vertices": [
    {
      "filename": "../assets/fixed/fixed.txt",
      "movement": [0, 0, 0],
      "coeff": 1.0
    }
  ],
  "external-objects": [],
  "g": [0, 0, 0],
  "init-vel": [0, 0, 0],
  "init-disp": [0, 0, 0],
  "scale": 1.0,
  "timestep": 0.01,
  "num-timestep": 1,
  "damping-params": [0, 0],
  "sim-type": "static",
  "contact-stiffness": 0.0,
  "contact-sample": 1,
  "contact-friction-coeff": 0.0,
  "contact-vel-eps": 1e-8,
  "solver-eps": 1e-8,
  "solver-max-iter": 2,
  "elastic-material": "stable-neo",
  "dump-interval": 1,
  "output": "result.obj"
})")
    );

    std::filesystem::create_directories(runnerDir);

    std::ostringstream cmd;
    cmd << "cd " << quotePath(runnerDir) << " && " << quotePath(runSimBin) << " --deterministic "
        << quotePath(std::filesystem::path("../project/config/sim.json")) << " > " << quotePath(logFile) << " 2>&1";

    ASSERT_EQ(runCommand(cmd.str()), 0);
    EXPECT_TRUE(std::filesystem::exists(outputFile));

    std::ifstream logStream(logFile);
    ASSERT_TRUE(logStream);
    const std::string logContent((std::istreambuf_iterator<char>(logStream)), std::istreambuf_iterator<char>());
    EXPECT_NE(logContent.find("Deterministic mode enabled"), std::string::npos);
}

TEST(RunSimToolTest, DeterministicModeProducesIdenticalOutputDirectoriesAcrossRepeatedRuns) {
    const std::filesystem::path runSimBin = getRunSimBinaryPath();
    ASSERT_FALSE(runSimBin.empty());
    ASSERT_TRUE(std::filesystem::exists(runSimBin));

    ScopedTempDir tempDir;

    const std::filesystem::path projectRoot   = tempDir.path() / "project";
    const std::filesystem::path runnerDir     = tempDir.path() / "runner";
    const std::filesystem::path configDir     = projectRoot / "config";
    const std::filesystem::path tetMeshFile   = projectRoot / "assets" / "tet" / "simple.veg";
    const std::filesystem::path surfaceFile   = projectRoot / "assets" / "surface" / "simple.obj";
    const std::filesystem::path fixedFile     = projectRoot / "assets" / "fixed" / "fixed.txt";
    const std::filesystem::path configFileA   = configDir / "sim-a.json";
    const std::filesystem::path configFileB   = configDir / "sim-b.json";
    const std::filesystem::path outputDirA    = configDir / "out-a";
    const std::filesystem::path outputDirB    = configDir / "out-b";

    ASSERT_TRUE(writeTextFile(tetMeshFile, R"(# Minimal tet mesh
*VERTICES
4 3 0 0
1 0 0 0
2 1 0 0
3 0 1 0
4 0 0 1

*ELEMENTS
TET
1 4 0
1 1 2 3 4
)")
    );

    ASSERT_TRUE(writeTextFile(surfaceFile, R"(v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
f 1 3 2
f 1 2 4
f 2 3 4
f 3 1 4
)")
    );

    ASSERT_TRUE(writeTextFile(fixedFile, "0\n"));

    auto writeDynamicConfig = [&](const std::filesystem::path& configFile, const char* outputDirName) {
        std::ostringstream content;
        content << R"({
  "tet-mesh": "../assets/tet/simple.veg",
  "surface-mesh": "../assets/surface/simple.obj",
  "fixed-vertices": [
    {
      "filename": "../assets/fixed/fixed.txt",
      "movement": [0, 0, 0],
      "coeff": 1.0
    }
  ],
  "external-objects": [],
  "g": [0, -9.81, 0],
  "init-vel": [0, 0, 0],
  "init-disp": [0, 0, 0],
  "scale": 1.0,
  "timestep": 0.01,
  "num-timestep": 3,
  "damping-params": [0, 0],
  "sim-type": "dynamic",
  "contact-stiffness": 0.0,
  "contact-sample": 1,
  "contact-friction-coeff": 0.0,
  "contact-vel-eps": 1e-8,
  "solver-eps": 1e-8,
  "solver-max-iter": 20,
  "elastic-material": "stable-neo",
  "dump-interval": 1,
  "deterministic": true,
  "output": ")" << outputDirName << R"("
})";
        return writeTextFile(configFile, content.str());
    };

    ASSERT_TRUE(writeDynamicConfig(configFileA, "out-a"));
    ASSERT_TRUE(writeDynamicConfig(configFileB, "out-b"));

    std::filesystem::create_directories(runnerDir);

    std::ostringstream cmdA;
    cmdA << "cd " << quotePath(runnerDir) << " && " << quotePath(runSimBin) << " "
         << quotePath(std::filesystem::path("../project/config/sim-a.json"));
    ASSERT_EQ(runCommand(cmdA.str()), 0);
    ASSERT_TRUE(std::filesystem::exists(outputDirA));

    std::ostringstream cmdB;
    cmdB << "cd " << quotePath(runnerDir) << " && " << quotePath(runSimBin) << " "
         << quotePath(std::filesystem::path("../project/config/sim-b.json"));
    ASSERT_EQ(runCommand(cmdB.str()), 0);
    ASSERT_TRUE(std::filesystem::exists(outputDirB));

    expectDirectoriesEqual(outputDirA, outputDirB);
}

}  // namespace test
