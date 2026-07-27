#pragma once

#include "material/import/importedMaterialCatalog.h"

namespace pgo
{
namespace VolumetricMeshes
{
class VolumetricMesh;
}
namespace SolidDeformationModel
{

ImportedMaterialCatalog importVolumeMaterialCatalog(
  const VolumetricMeshes::VolumetricMesh &volumeMesh);

}  // namespace SolidDeformationModel
}  // namespace pgo
