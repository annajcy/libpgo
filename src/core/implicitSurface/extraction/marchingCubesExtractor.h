#pragma once

#include "triMeshGeo.h"
#include "fields/GridField.h"

namespace pgo::ImplicitSurface {

struct MarchingCubesOptions {
  double isoOffset = 0.0;
};

void extractMarchingCubes(const GridField &field, const MarchingCubesOptions &options,
  Mesh::TriMeshGeo &outMesh);

}  // namespace pgo::ImplicitSurface
