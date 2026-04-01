#pragma once

#include "cubicMesh.h"
#include "tetMesh.h"
#include "volumetricMesh.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <vector>

namespace pgo::Mesh::test {

inline pgo::VolumetricMeshes::MaterialRecord makeEnuMaterial(const std::string& name, double density, double E,
                                                             double nu) {
    return pgo::VolumetricMeshes::MaterialRecord{name, pgo::VolumetricMeshes::EnuMaterialData{density, E, nu}};
}

inline pgo::VolumetricMeshes::MaterialRecord makeOrthotropicMaterial(const std::string& name, double density,
                                                                     double E1, double E2, double E3, double nu12,
                                                                     double nu23, double nu31, double G12, double G23,
                                                                     double G31,
                                                                     const std::array<double, 9>& rotation) {
    return pgo::VolumetricMeshes::MaterialRecord{
        name, pgo::VolumetricMeshes::OrthotropicMaterialData{density, E1, E2, E3, nu12, nu23, nu31, G12, G23, G31,
                                                             rotation}};
}

inline pgo::VolumetricMeshes::MaterialRecord makeMooneyRivlinMaterial(const std::string& name, double density,
                                                                      double mu01, double mu10, double v1) {
    return pgo::VolumetricMeshes::MaterialRecord{
        name, pgo::VolumetricMeshes::MooneyRivlinMaterialData{density, mu01, mu10, v1}};
}

inline std::filesystem::path uniqueTempPath(const std::string& suffix) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() / ("libpgo_core_scene_" + std::to_string(stamp) + suffix);
}

inline void expectVertexVectorsEqual(const std::vector<pgo::Vec3d>& lhs, const std::vector<pgo::Vec3d>& rhs) {
    ASSERT_EQ(lhs.size(), rhs.size());
    for (size_t i = 0; i < lhs.size(); ++i) {
        EXPECT_DOUBLE_EQ(lhs[i][0], rhs[i][0]);
        EXPECT_DOUBLE_EQ(lhs[i][1], rhs[i][1]);
        EXPECT_DOUBLE_EQ(lhs[i][2], rhs[i][2]);
    }
}

inline std::vector<std::byte> readBinaryFile(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    EXPECT_TRUE(stream.is_open());
    if (!stream.is_open()) {
        return {};
    }

    stream.seekg(0, std::ios::end);
    const auto size = stream.tellg();
    stream.seekg(0, std::ios::beg);

    std::vector<char> buffer(static_cast<size_t>(size));
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    EXPECT_TRUE(stream.good() || stream.eof());

    std::vector<std::byte> bytes(buffer.size());
    for (size_t i = 0; i < buffer.size(); ++i) {
        bytes[i] = static_cast<std::byte>(buffer[i]);
    }
    return bytes;
}

inline void writeTextFile(const std::filesystem::path& path, const std::string& contents) {
    std::ofstream stream(path);
    ASSERT_TRUE(stream.is_open());
    if (!stream.is_open()) {
        return;
    }

    stream << contents;
    stream.flush();
    EXPECT_TRUE(stream.good());
}

inline void expectMeshGeometryEqual(const pgo::VolumetricMeshes::VolumetricMesh& actual,
                                    const pgo::VolumetricMeshes::VolumetricMesh& expected) {
    ASSERT_EQ(actual.getElementType(), expected.getElementType());
    ASSERT_EQ(actual.getNumVertices(), expected.getNumVertices());
    ASSERT_EQ(actual.getNumElements(), expected.getNumElements());
    ASSERT_EQ(actual.getNumElementVertices(), expected.getNumElementVertices());

    expectVertexVectorsEqual(std::vector<pgo::Vec3d>(actual.getVertices().begin(), actual.getVertices().end()),
                             std::vector<pgo::Vec3d>(expected.getVertices().begin(), expected.getVertices().end()));
    EXPECT_EQ(std::vector<int>(actual.getElements().begin(), actual.getElements().end()),
              std::vector<int>(expected.getElements().begin(), expected.getElements().end()));
}

inline void expectWeightsNormalized(const double* weights, int count) {
    double sum = 0.0;
    for (int i = 0; i < count; ++i) {
        EXPECT_GE(weights[i], -1e-12);
        EXPECT_LE(weights[i], 1.0 + 1e-12);
        sum += weights[i];
    }
    EXPECT_NEAR(sum, 1.0, 1e-12);
}

inline void expectAllElementsSetCoversMesh(const pgo::VolumetricMeshes::VolumetricMesh& mesh) {
    ASSERT_GT(mesh.getNumSets(), 0);
    const auto& all_elements = mesh.getSet(0).getElements();
    EXPECT_EQ(mesh.getSet(0).getName(), "allElements");
    EXPECT_EQ(static_cast<int>(all_elements.size()), mesh.getNumElements());
    for (int element = 0; element < mesh.getNumElements(); ++element) {
        EXPECT_NE(all_elements.find(element), all_elements.end());
    }
}

inline pgo::VolumetricMeshes::TetMesh makeSingleTetMesh(double E = 1234.0, double nu = 0.31, double density = 7.5) {
    const std::vector<pgo::Vec3d> vertices{
        pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(0.0, 1.0, 0.0), pgo::Vec3d(0.0, 0.0, 1.0)};
    const std::vector<pgo::Vec4i> tets{pgo::Vec4i(0, 1, 2, 3)};
    return pgo::VolumetricMeshes::TetMesh(vertices, tets, E, nu, density);
}

inline pgo::VolumetricMeshes::TetMesh makeTwoTetMeshWithDistinctMaterials() {
    const std::vector<pgo::Vec3d> vertices{
        pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(0.0, 1.0, 0.0), pgo::Vec3d(0.0, 0.0, 1.0),
        pgo::Vec3d(3.0, 0.0, 0.0), pgo::Vec3d(4.0, 0.0, 0.0), pgo::Vec3d(3.0, 1.0, 0.0), pgo::Vec3d(3.0, 0.0, 1.0)};
    const std::vector<pgo::Vec4i> tets{pgo::Vec4i(0, 1, 2, 3), pgo::Vec4i(4, 5, 6, 7)};

    pgo::VolumetricMeshes::TetMesh mesh(vertices, tets, 1234.0, 0.31, 7.5);

    const auto second_material = makeEnuMaterial("secondMaterial", 9.25, 4321.0, 0.21);
    pgo::VolumetricMeshes::ElementSet second_set("secondElement", std::set<int>{1});
    mesh.addMaterial(second_material, second_set, true, true);

    return mesh;
}

inline pgo::VolumetricMeshes::CubicMesh makeSingleCubeMesh(double E = 500.0, double nu = 0.25, double density = 3.0) {
    const std::vector<pgo::Vec3d> vertices{pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0),
                                           pgo::Vec3d(1.0, 1.0, 0.0), pgo::Vec3d(0.0, 1.0, 0.0),
                                           pgo::Vec3d(0.0, 0.0, 1.0), pgo::Vec3d(1.0, 0.0, 1.0),
                                           pgo::Vec3d(1.0, 1.0, 1.0), pgo::Vec3d(0.0, 1.0, 1.0)};
    const std::vector<pgo::VolumetricMeshes::CubicElement> elements{{0, 1, 2, 3, 4, 5, 6, 7}};
    return pgo::VolumetricMeshes::CubicMesh(vertices, elements, E, nu, density);
}

inline pgo::VolumetricMeshes::CubicMesh makeTwoCubeMeshWithDistinctMaterials() {
    const std::vector<pgo::Vec3d> vertices{
        pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(1.0, 1.0, 0.0), pgo::Vec3d(0.0, 1.0, 0.0),
        pgo::Vec3d(0.0, 0.0, 1.0), pgo::Vec3d(1.0, 0.0, 1.0), pgo::Vec3d(1.0, 1.0, 1.0), pgo::Vec3d(0.0, 1.0, 1.0),
        pgo::Vec3d(3.0, 0.0, 0.0), pgo::Vec3d(4.0, 0.0, 0.0), pgo::Vec3d(4.0, 1.0, 0.0), pgo::Vec3d(3.0, 1.0, 0.0),
        pgo::Vec3d(3.0, 0.0, 1.0), pgo::Vec3d(4.0, 0.0, 1.0), pgo::Vec3d(4.0, 1.0, 1.0), pgo::Vec3d(3.0, 1.0, 1.0)};
    const std::vector<pgo::VolumetricMeshes::CubicElement> elements{{0, 1, 2, 3, 4, 5, 6, 7},
                                                                     {8, 9, 10, 11, 12, 13, 14, 15}};

    pgo::VolumetricMeshes::CubicMesh mesh(vertices, elements, 500.0, 0.25, 3.0);

    const auto second_material = makeEnuMaterial("secondMaterial", 6.5, 1800.0, 0.33);
    pgo::VolumetricMeshes::ElementSet second_set("secondElement", std::set<int>{1});
    mesh.addMaterial(second_material, second_set, true, true);

    return mesh;
}

}  // namespace pgo::Mesh::test
