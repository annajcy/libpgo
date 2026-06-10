#pragma once

#include "cubicFormulation.h"

namespace pgo
{
namespace SolidDeformationModel
{

class CubicTricubicHermiteFormulation : public CubicFormulation
{
public:
  CubicTricubicHermiteFormulation();
  std::string_view getName() const override;
  int getNodesPerElement() const override;
  int getLocalDofs() const override;

  std::unique_ptr<DofLayout> createDofLayout(const SimulationMesh &mesh) const override;
  EigenSupport::VXd buildGlobalRestDofs(const SimulationMesh &mesh) const override;

  EigenSupport::SpMatD buildSurfaceEmbeddingMatrix(
    const VolumetricMeshes::VolumetricMesh &mesh,
    const EigenSupport::MXd &surfaceVertices) const override;

  std::unique_ptr<DeformationModel> createElement(
    const SimulationMesh &mesh, int ele,
    std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel) const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
