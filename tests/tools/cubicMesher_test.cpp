#include "cubicMesh.h"
#include "triMeshGeo.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#ifndef PGO_TEST_CUBIC_MESHER_BIN
#define PGO_TEST_CUBIC_MESHER_BIN ""
#endif

namespace test {

namespace {
std::string quotePath(const std::filesystem::path& p) {
    return "\"" + p.string() + "\"";
}

std::string shellExecutable(const std::filesystem::path& p) {
#ifdef _WIN32
    return "call " + quotePath(p);
#else
    return quotePath(p);
#endif
}

std::filesystem::path getCubicMesherBinaryPath() {
    const char* bin = std::getenv("CUBIC_MESHER_BIN");
    if (bin != nullptr && !std::string(bin).empty()) {
        return std::filesystem::path(bin);
    }

    if (std::string(PGO_TEST_CUBIC_MESHER_BIN).empty()) {
        return {};
    }

    return std::filesystem::path(PGO_TEST_CUBIC_MESHER_BIN);
}

int runCommand(const std::string& command) {
    return std::system(command.c_str());
}

#if !defined(PGO_HAS_CGAL)
std::string readTextFile(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

struct CommandResult {
    int         exitCode = 0;
    std::string output;
};

CommandResult runCommandCaptureOutput(const std::string& command, const std::filesystem::path& capturePath) {
    std::ostringstream wrappedCommand;
    wrappedCommand << command << " > " << quotePath(capturePath) << " 2>&1";
    return {runCommand(wrappedCommand.str()), readTextFile(capturePath)};
}
#endif

#if defined(PGO_HAS_CGAL)
void writeUnitCubeObj(const std::filesystem::path& outputObj) {
    std::ofstream out(outputObj);
    out << "v 0 0 0\n";
    out << "v 1 0 0\n";
    out << "v 1 1 0\n";
    out << "v 0 1 0\n";
    out << "v 0 0 1\n";
    out << "v 1 0 1\n";
    out << "v 1 1 1\n";
    out << "v 0 1 1\n";

    out << "f 1 3 2\n";
    out << "f 1 4 3\n";
    out << "f 5 6 7\n";
    out << "f 5 7 8\n";
    out << "f 1 2 6\n";
    out << "f 1 6 5\n";
    out << "f 4 8 7\n";
    out << "f 4 7 3\n";
    out << "f 1 5 8\n";
    out << "f 1 8 4\n";
    out << "f 2 3 7\n";
    out << "f 2 7 6\n";
}
#endif

class ScopedTempDir {
public:
    ScopedTempDir() {
        const auto base = std::filesystem::temp_directory_path();
        for (int attempt = 0; attempt < 32; attempt++) {
            const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
            path_            = base / ("libpgo-cubicMesher-test-" + std::to_string(stamp) + "-" +
                            std::to_string(std::rand()) + "-" + std::to_string(attempt));
            std::error_code ec;
            if (std::filesystem::create_directories(path_, ec)) {
                return;
            }
        }

        throw std::runtime_error("Failed to create a unique temporary directory for cubicMesher tests");
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

TEST(CubicMesherTool, UniformResolution2ProducesExpectedCubicMesh) {
    const std::filesystem::path cubicMesherBin = getCubicMesherBinaryPath();
    ASSERT_FALSE(cubicMesherBin.empty());
    ASSERT_TRUE(std::filesystem::exists(cubicMesherBin));

    ScopedTempDir               tempDir;
    const std::filesystem::path outputVeg = tempDir.path() / "r2.veg";

    std::ostringstream cmd;
    cmd << shellExecutable(cubicMesherBin) << " uniform --resolution 2 --output-mesh " << quotePath(outputVeg)
        << " --E 1e6 --nu 0.45 --density 1000";

    ASSERT_EQ(runCommand(cmd.str()), 0);
    ASSERT_TRUE(std::filesystem::exists(outputVeg));

    pgo::VolumetricMeshes::CubicMesh mesh(outputVeg.string().c_str());
    EXPECT_EQ(mesh.getNumVertices(), 27);
    EXPECT_EQ(mesh.getNumElements(), 8);
}

TEST(CubicMesherTool, UniformResolution4CanExportSurfaceMesh) {
    const std::filesystem::path cubicMesherBin = getCubicMesherBinaryPath();
    ASSERT_FALSE(cubicMesherBin.empty());
    ASSERT_TRUE(std::filesystem::exists(cubicMesherBin));

    ScopedTempDir               tempDir;
    const std::filesystem::path outputVeg = tempDir.path() / "r4.veg";
    const std::filesystem::path outputObj = tempDir.path() / "r4.obj";

    std::ostringstream cmd;
    cmd << shellExecutable(cubicMesherBin) << " uniform --resolution 4 --output-mesh " << quotePath(outputVeg)
        << " --output-surface " << quotePath(outputObj) << " --E 1e6 --nu 0.45 --density 1000";

    ASSERT_EQ(runCommand(cmd.str()), 0);
    ASSERT_TRUE(std::filesystem::exists(outputVeg));
    ASSERT_TRUE(std::filesystem::exists(outputObj));

    pgo::VolumetricMeshes::CubicMesh mesh(outputVeg.string().c_str());
    EXPECT_EQ(mesh.getNumVertices(), 125);
    EXPECT_EQ(mesh.getNumElements(), 64);

    pgo::Mesh::TriMeshGeo surface;
    ASSERT_TRUE(surface.load(outputObj.string()));
    EXPECT_EQ(surface.numTriangles(), 192);
    EXPECT_GT(surface.numVertices(), 0);
}

TEST(CubicMesherTool, UniformSizeScalesOutputBoundingBox) {
    const std::filesystem::path cubicMesherBin = getCubicMesherBinaryPath();
    ASSERT_FALSE(cubicMesherBin.empty());
    ASSERT_TRUE(std::filesystem::exists(cubicMesherBin));

    ScopedTempDir               tempDir;
    const std::filesystem::path outputVeg = tempDir.path() / "size2.veg";

    std::ostringstream cmd;
    cmd << shellExecutable(cubicMesherBin) << " uniform --resolution 2 --size 2.0 --output-mesh "
        << quotePath(outputVeg) << " --E 1e6 --nu 0.45 --density 1000";

    ASSERT_EQ(runCommand(cmd.str()), 0);
    ASSERT_TRUE(std::filesystem::exists(outputVeg));

    pgo::VolumetricMeshes::CubicMesh mesh(outputVeg.string().c_str());
    EXPECT_EQ(mesh.getNumVertices(), 27);
    EXPECT_EQ(mesh.getNumElements(), 8);

    const pgo::Mesh::BoundingBox bb    = mesh.getBoundingBox();
    const pgo::Vec3d             sides = bb.sides();
    EXPECT_DOUBLE_EQ(sides[0], 2.0);
    EXPECT_DOUBLE_EQ(sides[1], 2.0);
    EXPECT_DOUBLE_EQ(sides[2], 2.0);
    EXPECT_DOUBLE_EQ(bb.center()[0], 0.0);
    EXPECT_DOUBLE_EQ(bb.center()[1], 0.0);
    EXPECT_DOUBLE_EQ(bb.center()[2], 0.0);
}

TEST(CubicMesherTool, UniformOffsetMovesOutputBoundingBoxCenter) {
    const std::filesystem::path cubicMesherBin = getCubicMesherBinaryPath();
    ASSERT_FALSE(cubicMesherBin.empty());
    ASSERT_TRUE(std::filesystem::exists(cubicMesherBin));

    ScopedTempDir               tempDir;
    const std::filesystem::path outputVeg = tempDir.path() / "offset.veg";

    std::ostringstream cmd;
    cmd << shellExecutable(cubicMesherBin) << " uniform --resolution 2 --size 2.0 --offset 1.5 -2.0 0.25"
        << " --output-mesh " << quotePath(outputVeg) << " --E 1e6 --nu 0.45 --density 1000";

    ASSERT_EQ(runCommand(cmd.str()), 0);
    ASSERT_TRUE(std::filesystem::exists(outputVeg));

    pgo::VolumetricMeshes::CubicMesh mesh(outputVeg.string().c_str());
    const pgo::Mesh::BoundingBox     bb    = mesh.getBoundingBox();
    const pgo::Vec3d                 sides = bb.sides();
    EXPECT_DOUBLE_EQ(sides[0], 2.0);
    EXPECT_DOUBLE_EQ(sides[1], 2.0);
    EXPECT_DOUBLE_EQ(sides[2], 2.0);
    EXPECT_DOUBLE_EQ(bb.center()[0], 1.5);
    EXPECT_DOUBLE_EQ(bb.center()[1], -2.0);
    EXPECT_DOUBLE_EQ(bb.center()[2], 0.25);
}

TEST(CubicMesherTool, InvalidResolutionFails) {
    const std::filesystem::path cubicMesherBin = getCubicMesherBinaryPath();
    ASSERT_FALSE(cubicMesherBin.empty());
    ASSERT_TRUE(std::filesystem::exists(cubicMesherBin));

    ScopedTempDir               tempDir;
    const std::filesystem::path outputVeg = tempDir.path() / "invalid.veg";

    std::ostringstream cmd;
    cmd << shellExecutable(cubicMesherBin) << " uniform --resolution 0 --output-mesh " << quotePath(outputVeg);

    EXPECT_NE(runCommand(cmd.str()), 0);
    EXPECT_FALSE(std::filesystem::exists(outputVeg));
}

TEST(CubicMesherTool, InvalidSizeFails) {
    const std::filesystem::path cubicMesherBin = getCubicMesherBinaryPath();
    ASSERT_FALSE(cubicMesherBin.empty());
    ASSERT_TRUE(std::filesystem::exists(cubicMesherBin));

    ScopedTempDir               tempDir;
    const std::filesystem::path outputVeg = tempDir.path() / "invalid_size.veg";

    std::ostringstream cmd;
    cmd << shellExecutable(cubicMesherBin) << " uniform --resolution 2 --size 0 --output-mesh " << quotePath(outputVeg);

    EXPECT_NE(runCommand(cmd.str()), 0);
    EXPECT_FALSE(std::filesystem::exists(outputVeg));
}

#if defined(PGO_HAS_CGAL)
TEST(CubicMesherTool, TriangleMeshCenterVoxelizationProducesExpectedCubicMesh) {
    const std::filesystem::path cubicMesherBin = getCubicMesherBinaryPath();
    ASSERT_FALSE(cubicMesherBin.empty());
    ASSERT_TRUE(std::filesystem::exists(cubicMesherBin));

    ScopedTempDir               tempDir;
    const std::filesystem::path inputObj   = tempDir.path() / "cube.obj";
    const std::filesystem::path outputVeg  = tempDir.path() / "cube.veg";
    const std::filesystem::path outputSurf = tempDir.path() / "cube_surface.obj";
    writeUnitCubeObj(inputObj);
    ASSERT_TRUE(std::filesystem::exists(inputObj));

    std::ostringstream cmd;
    cmd << shellExecutable(cubicMesherBin) << " triangle-mesh --input-mesh " << quotePath(inputObj)
        << " --resolution 4 --padding-voxels 1 --classify-mode center --output-mesh " << quotePath(outputVeg)
        << " --output-surface " << quotePath(outputSurf) << " --E 1e6 --nu 0.45 --density 1000";

    ASSERT_EQ(runCommand(cmd.str()), 0);
    ASSERT_TRUE(std::filesystem::exists(outputVeg));
    ASSERT_TRUE(std::filesystem::exists(outputSurf));

    pgo::VolumetricMeshes::CubicMesh mesh(outputVeg.string().c_str());
    EXPECT_EQ(mesh.getNumVertices(), 27);
    EXPECT_EQ(mesh.getNumElements(), 8);

    const pgo::Mesh::BoundingBox bb    = mesh.getBoundingBox();
    const pgo::Vec3d             sides = bb.sides();
    EXPECT_DOUBLE_EQ(sides[0], 1.0);
    EXPECT_DOUBLE_EQ(sides[1], 1.0);
    EXPECT_DOUBLE_EQ(sides[2], 1.0);
    EXPECT_DOUBLE_EQ(bb.center()[0], 0.5);
    EXPECT_DOUBLE_EQ(bb.center()[1], 0.5);
    EXPECT_DOUBLE_EQ(bb.center()[2], 0.5);

    pgo::Mesh::TriMeshGeo surface;
    ASSERT_TRUE(surface.load(outputSurf.string()));
    EXPECT_EQ(surface.numTriangles(), 48);
    EXPECT_GT(surface.numVertices(), 0);
}

TEST(CubicMesherTool, TriangleMeshScaleAndOffsetTransformOutputBoundingBox) {
    const std::filesystem::path cubicMesherBin = getCubicMesherBinaryPath();
    ASSERT_FALSE(cubicMesherBin.empty());
    ASSERT_TRUE(std::filesystem::exists(cubicMesherBin));

    ScopedTempDir               tempDir;
    const std::filesystem::path inputObj  = tempDir.path() / "cube_transform.obj";
    const std::filesystem::path outputVeg = tempDir.path() / "cube_transform.veg";
    writeUnitCubeObj(inputObj);
    ASSERT_TRUE(std::filesystem::exists(inputObj));

    std::ostringstream cmd;
    cmd << shellExecutable(cubicMesherBin) << " triangle-mesh --input-mesh " << quotePath(inputObj)
        << " --resolution 4 --padding-voxels 1 --classify-mode center --scale 2.0 --offset 1.5 -2.0 0.25"
        << " --output-mesh " << quotePath(outputVeg) << " --E 1e6 --nu 0.45 --density 1000";

    ASSERT_EQ(runCommand(cmd.str()), 0);
    ASSERT_TRUE(std::filesystem::exists(outputVeg));

    pgo::VolumetricMeshes::CubicMesh mesh(outputVeg.string().c_str());
    const pgo::Mesh::BoundingBox     bb    = mesh.getBoundingBox();
    const pgo::Vec3d                 sides = bb.sides();
    EXPECT_DOUBLE_EQ(sides[0], 2.0);
    EXPECT_DOUBLE_EQ(sides[1], 2.0);
    EXPECT_DOUBLE_EQ(sides[2], 2.0);
    EXPECT_DOUBLE_EQ(bb.center()[0], 2.5);
    EXPECT_DOUBLE_EQ(bb.center()[1], -1.0);
    EXPECT_DOUBLE_EQ(bb.center()[2], 1.25);
}

TEST(CubicMesherTool, TriangleMeshInvalidScaleFails) {
    const std::filesystem::path cubicMesherBin = getCubicMesherBinaryPath();
    ASSERT_FALSE(cubicMesherBin.empty());
    ASSERT_TRUE(std::filesystem::exists(cubicMesherBin));

    ScopedTempDir               tempDir;
    const std::filesystem::path inputObj  = tempDir.path() / "cube_invalid_scale.obj";
    const std::filesystem::path outputVeg = tempDir.path() / "cube_invalid_scale.veg";
    writeUnitCubeObj(inputObj);
    ASSERT_TRUE(std::filesystem::exists(inputObj));

    std::ostringstream cmd;
    cmd << shellExecutable(cubicMesherBin) << " triangle-mesh --input-mesh " << quotePath(inputObj)
        << " --resolution 4 --scale 0 --output-mesh " << quotePath(outputVeg);

    EXPECT_NE(runCommand(cmd.str()), 0);
    EXPECT_FALSE(std::filesystem::exists(outputVeg));
}
#else
TEST(CubicMesherTool, TriangleMeshRequiresGeometryStackWhenUnavailable) {
    const std::filesystem::path cubicMesherBin = getCubicMesherBinaryPath();
    ASSERT_FALSE(cubicMesherBin.empty());
    ASSERT_TRUE(std::filesystem::exists(cubicMesherBin));

    ScopedTempDir               tempDir;
    const std::filesystem::path outputVeg   = tempDir.path() / "triangle.veg";
    const std::filesystem::path outputLog   = tempDir.path() / "triangle.log";
    const std::filesystem::path missingMesh = tempDir.path() / "missing.obj";

    std::ostringstream cmd;
    cmd << shellExecutable(cubicMesherBin) << " triangle-mesh --input-mesh " << quotePath(missingMesh)
        << " --resolution 8 --output-mesh " << quotePath(outputVeg);

    const CommandResult result = runCommandCaptureOutput(cmd.str(), outputLog);
    EXPECT_NE(result.exitCode, 0);
    EXPECT_FALSE(std::filesystem::exists(outputVeg));
    EXPECT_NE(result.output.find("geometry stack is not supported."), std::string::npos);
}
#endif

}  // namespace test
