#pragma once

#include "cubicMesh.h"

#include <string>

namespace cubic_mesher {

bool saveAndValidateCubicMesh(const pgo::VolumetricMeshes::CubicMesh& cubicMesh, const std::string& outputMesh);

bool writeSurfaceMesh(const pgo::VolumetricMeshes::CubicMesh& cubicMesh, const std::string& outputSurface);

}  // namespace cubic_mesher
