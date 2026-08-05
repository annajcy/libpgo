#pragma once

#include "formulations/formulation/formulation.h"

#include "EigenDef.h"
#include <array>

namespace pgo
{
namespace SolidDeformationModel
{

class ShellElementMapping;

class ShellFormulation : public Formulation
{
public:
  virtual std::unique_ptr<ShellElementMapping> createElementMapping(
    const EigenSupport::V18d &restX, const std::array<bool, 6> &hasVtx) const = 0;

  std::unique_ptr<DeformationElement> createElement(
    const SimulationMesh &mesh, int ele,
    std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel,
    DeformationElementConstructionOptions options = {}) const override;

  SimulationMeshType compatibleMeshType() const override;

  // Lumped shell mass / body force: per triangle, arealDensity(e)*area_e/3 to
  // each of the three corner vertices. The Koiter 6-vertex stencil only
  // affects bending energy; displacement DOFs are 3 per vertex.
  EigenSupport::SpMatD buildMassMatrix(
    const SimulationMesh &mesh,
    EigenSupport::ConstRefVecXd elementArealDensities) const;
  EigenSupport::VXd buildBodyForce(
    const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
    EigenSupport::ConstRefVecXd elementArealDensities) const;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
