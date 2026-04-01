#include "cubicMesh.h"
#include "triMeshGeo.h"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
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

class ScopedTempDir {
public:
    ScopedTempDir() {
        path_ = std::filesystem::temp_directory_path() / std::filesystem::path("libpgo-cubicMesher-test");
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

}  // namespace test
