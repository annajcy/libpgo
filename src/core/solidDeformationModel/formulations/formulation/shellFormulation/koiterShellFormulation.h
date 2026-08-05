#pragma once

#include "formulations/formulation/formulation.h"

namespace pgo
{
namespace SolidDeformationModel
{

class KoiterShellFormulation final : public Formulation
{
public:
  std::string_view getName() const override;
  int numBasisFunctionsPerElement() const override;
  int getLocalDofs() const override;
  SimulationMeshType compatibleMeshType() const override;

  std::unique_ptr<DeformationElement> createElement(
    const SimulationMesh &mesh, int ele,
    std::unique_ptr<ElasticModel> elasticModel,
    std::unique_ptr<PlasticModel> plasticModel,
    DeformationElementConstructionOptions options = {}) const override;

  EigenSupport::SpMatD buildMassMatrix(
    const SimulationMesh &mesh,
    EigenSupport::ConstRefVecXd elementArealDensities) const;
  EigenSupport::VXd buildBodyForce(
    const SimulationMesh &mesh,
    const EigenSupport::V3d &acceleration,
    EigenSupport::ConstRefVecXd elementArealDensities) const;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
