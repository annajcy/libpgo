#pragma once

#include "volumetricMesh.h"

#include <memory>

namespace pgo::VolumetricMeshes {

class TetMesh;
class CubicMesh;

namespace io {

std::unique_ptr<TetMesh> load_tet(const std::filesystem::path& filename,
                                  VolumetricMesh::FileFormatType fileFormat =
                                      VolumetricMesh::FileFormatType::ByExtension,
                                  int verbose = 1);
std::unique_ptr<CubicMesh> load_cubic(const std::filesystem::path& filename,
                                      VolumetricMesh::FileFormatType fileFormat =
                                          VolumetricMesh::FileFormatType::ByExtension,
                                      int verbose = 1);

}  // namespace io

}  // namespace pgo::VolumetricMeshes
