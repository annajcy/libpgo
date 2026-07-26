#pragma once

#include "volumetricFormulation.h"

namespace pgo
{
namespace SolidDeformationModel
{

class CubicFormulation : public VolumetricFormulation
{
public:
  using VolumetricFormulation::VolumetricFormulation;
  SimulationMeshType compatibleMeshType() const override;
  EigenSupport::SpMatD buildSurfaceEmbeddingMatrix(
    const VolumetricMeshes::VolumetricMesh &mesh,
    const EigenSupport::MXd &surfaceVertices) const override;
  static EigenSupport::V3d clampedParametricCoordinates(
    const EigenSupport::V8d &weights);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
