#pragma once

#include "meshData.h"

#include <array>
#include <filesystem>
#include <string>
#include <variant>
#include <vector>

namespace pgo::VolumetricMeshes
{

struct VegENuMaterialPayload
{
  std::string name;
  double density = 1000.0;
  double E = 1e9;
  double nu = 0.45;
};

struct VegMooneyRivlinMaterialPayload
{
  std::string name;
  double density = 1000.0;
  double mu01 = 0.0;
  double mu10 = 0.0;
  double v1 = 0.0;
};

struct VegOrthotropicMaterialPayload
{
  std::string name;
  double density = 1000.0;
  double E1 = 0.0;
  double E2 = 0.0;
  double E3 = 0.0;
  double nu12 = 0.0;
  double nu23 = 0.0;
  double nu31 = 0.0;
  double G12 = 0.0;
  double G23 = 0.0;
  double G31 = 0.0;
  std::array<double, 9> R{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
};

using VegMaterialPayload = std::variant<
  VegENuMaterialPayload,
  VegMooneyRivlinMaterialPayload,
  VegOrthotropicMaterialPayload>;

struct VegSetPayload
{
  std::string name;
  std::vector<int> elements;
};

struct VegRegionPayload
{
  int materialIndex = 0;
  int setIndex = 0;
};

using VegMeshData = std::variant<pgo::Mesh::TetMeshData, pgo::Mesh::CubicMeshData>;

struct VegFilePayload
{
  VegMeshData meshData;
  std::vector<VegMaterialPayload> materials;
  std::vector<VegSetPayload> sets;
  std::vector<VegRegionPayload> regions;
};

VegFilePayload readVegFile(const std::filesystem::path &path);
void writeVegFile(const std::filesystem::path &path, const VegFilePayload &payload);

}  // namespace pgo::VolumetricMeshes
