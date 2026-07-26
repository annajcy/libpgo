/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "meshLinearAlgebra.h"

#include <array>
#include <span>

namespace pgo
{
namespace SolidDeformationModel
{
void computeTetMeshOccupation(std::span<const Vec3d> tetVertices, std::span<const Vec4i> tets,
  std::span<const Vec3d> surfaceVertices, std::span<const Vec3i> triangles,
  double minThreshold, int sampleCount, std::span<double> weights);

void computeTetMeshOccupation(std::span<const Vec3d> tetVertices, std::span<const Vec4i> tets,
  const std::array<Vec3d, 2> &bb, double minThreshold, int sampleCount, std::span<double> weights);
}  // namespace SolidDeformationModel
}  // namespace pgo
