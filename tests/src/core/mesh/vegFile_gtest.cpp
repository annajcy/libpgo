#include <gtest/gtest.h>

#include "tetMesh.h"
#include "vegFile.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshMooneyRivlinMaterial.h"
#include "volumetricMeshOrthotropicMaterial.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <set>
#include <string>
#include <vector>

namespace
{
using pgo::Vec3d;
using pgo::VolumetricMeshes::TetMesh;
using pgo::VolumetricMeshes::VegENuMaterialPayload;
using pgo::VolumetricMeshes::VegFilePayload;
using pgo::VolumetricMeshes::VegMooneyRivlinMaterialPayload;
using pgo::VolumetricMeshes::VegOrthotropicMaterialPayload;
using pgo::VolumetricMeshes::VegRegionPayload;
using pgo::VolumetricMeshes::VegSetPayload;
using VM = pgo::VolumetricMeshes::VolumetricMesh;

std::filesystem::path makeTempPath(const std::string &extension)
{
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
    ("libpgo_vegFile_gtest_" + std::to_string(stamp) + extension);
}

VegFilePayload makePayload()
{
  VegFilePayload payload;
  std::vector<Vec3d> vertices{
    Vec3d(0.0, 0.0, 0.0),
    Vec3d(1.0, 0.0, 0.0),
    Vec3d(0.0, 1.0, 0.0),
    Vec3d(0.0, 0.0, 1.0),
    Vec3d(1.0, 1.0, 1.0),
  };
  std::vector<int> elements{ 0, 1, 2, 3, 1, 2, 3, 4 };
  payload.meshData = pgo::Mesh::TetMeshData::fromFlatElements(std::move(vertices), std::move(elements));
  payload.materials.push_back(VegENuMaterialPayload{ "soft", 1000.0, 2.0e6, 0.35 });
  payload.materials.push_back(VegMooneyRivlinMaterialPayload{ "insert", 1200.0, 3.0, 4.0, 0.2 });
  payload.materials.push_back(VegOrthotropicMaterialPayload{
    "unusedOrtho", 900.0,
    10.0, 11.0, 12.0,
    0.1, 0.2, 0.3,
    2.0, 3.0, 4.0,
    { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 } });
  payload.sets.push_back(VegSetPayload{ "allElements", { 0, 1 } });
  payload.sets.push_back(VegSetPayload{ "softSet", { 0 } });
  payload.sets.push_back(VegSetPayload{ "insertSet", { 1 } });
  payload.regions.push_back(VegRegionPayload{ 0, 1 });
  payload.regions.push_back(VegRegionPayload{ 1, 2 });
  return payload;
}

void expectPayloadMatches(const VegFilePayload &payload)
{
  ASSERT_TRUE(std::holds_alternative<pgo::Mesh::TetMeshData>(payload.meshData));
  const auto &meshData = std::get<pgo::Mesh::TetMeshData>(payload.meshData);
  EXPECT_EQ(meshData.numVertices(), 5);
  EXPECT_EQ(meshData.numElements(), 2);
  EXPECT_DOUBLE_EQ(meshData.positions()[4][0], 1.0);
  EXPECT_EQ(meshData.elementVtxID(1, 3), 4);

  ASSERT_EQ(payload.materials.size(), 3);
  ASSERT_TRUE(std::holds_alternative<VegENuMaterialPayload>(payload.materials[0]));
  EXPECT_DOUBLE_EQ(std::get<VegENuMaterialPayload>(payload.materials[0]).E, 2.0e6);
  ASSERT_TRUE(std::holds_alternative<VegMooneyRivlinMaterialPayload>(payload.materials[1]));
  EXPECT_DOUBLE_EQ(std::get<VegMooneyRivlinMaterialPayload>(payload.materials[1]).mu10, 4.0);
  ASSERT_TRUE(std::holds_alternative<VegOrthotropicMaterialPayload>(payload.materials[2]));
  EXPECT_DOUBLE_EQ(std::get<VegOrthotropicMaterialPayload>(payload.materials[2]).G31, 4.0);

  ASSERT_EQ(payload.sets.size(), 3);
  EXPECT_EQ(payload.sets[0].name, "allElements");
  EXPECT_EQ(payload.sets[0].elements, std::vector<int>({ 0, 1 }));
  EXPECT_EQ(payload.sets[1].name, "softSet");
  EXPECT_EQ(payload.sets[1].elements, std::vector<int>({ 0 }));
  EXPECT_EQ(payload.sets[2].name, "insertSet");
  EXPECT_EQ(payload.sets[2].elements, std::vector<int>({ 1 }));

  ASSERT_EQ(payload.regions.size(), 2);
  EXPECT_EQ(payload.regions[0].materialIndex, 0);
  EXPECT_EQ(payload.regions[0].setIndex, 1);
  EXPECT_EQ(payload.regions[1].materialIndex, 1);
  EXPECT_EQ(payload.regions[1].setIndex, 2);
}
}  // namespace

TEST(VegFileGTest, WritesAndReadsAsciiPayload)
{
  const auto path = makeTempPath(".veg");
  pgo::VolumetricMeshes::writeVegFile(path, makePayload());

  const VegFilePayload loaded = pgo::VolumetricMeshes::readVegFile(path);
  expectPayloadMatches(loaded);

  std::remove(path.string().c_str());
}

TEST(VegFileGTest, ReadsBinaryPayload)
{
  const auto binaryPath = makeTempPath(".vegb");
  std::vector<double> vertices{
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
    1.0, 1.0, 1.0,
  };
  std::vector<int> elements{ 0, 1, 2, 3, 1, 2, 3, 4 };
  VM::ENuMaterial soft("soft", 1000.0, 2.0e6, 0.35);
  VM::MooneyRivlinMaterial insert("insert", 1200.0, 3.0, 4.0, 0.2);
  double R[9]{ 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
  VM::OrthotropicMaterial unusedOrtho(
    "unusedOrtho", 900.0,
    10.0, 11.0, 12.0,
    0.1, 0.2, 0.3,
    2.0, 3.0, 4.0,
    R);
  std::vector<const VM::Material *> materials{ &soft, &insert, &unusedOrtho };
  std::vector<VM::Set> sets{
    VM::Set("allElements", std::set<int>{ 0, 1 }),
    VM::Set("softSet", std::set<int>{ 0 }),
    VM::Set("insertSet", std::set<int>{ 1 }),
  };
  std::vector<VM::Region> regions{ VM::Region(0, 1), VM::Region(1, 2) };

  TetMesh mesh(
    5, vertices.data(),
    2, elements.data(),
    static_cast<int>(materials.size()), materials.data(),
    static_cast<int>(sets.size()), sets.data(),
    static_cast<int>(regions.size()), regions.data());
  ASSERT_EQ(mesh.saveToBinary(binaryPath.string().c_str()), 0);

  const VegFilePayload loaded = pgo::VolumetricMeshes::readVegFile(binaryPath);
  expectPayloadMatches(loaded);

  std::remove(binaryPath.string().c_str());
}
