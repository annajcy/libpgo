#pragma once

#include "formulations/formulation/formulation.h"

#include "EigenDef.h"

namespace pgo
{
namespace SolidDeformationModel
{

class ShellElementMapping;
class ShellMassField;

class ShellFormulation : public Formulation
{
public:
  virtual std::unique_ptr<ShellElementMapping> createElementMapping(
    const double restX[18], const bool hasVtx[6]) const = 0;

  std::unique_ptr<DeformationModel> createElement(
    const SimulationMesh &mesh, int ele,
    std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel) const override;

  SimulationMeshType compatibleMeshType() const override;

  // Lumped shell mass / body force: per triangle, arealDensity(e)*area_e/3 to
  // each of the three corner vertices. The Koiter 6-vertex stencil only
  // affects bending energy; displacement DOFs are 3 per vertex.
  EigenSupport::SpMatD buildMassMatrix(
    const SimulationMesh &mesh, const ShellMassField &massField) const;
  EigenSupport::VXd buildBodyForce(
    const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
    const ShellMassField &massField) const;

  // d f_g / d b for a parameter-dependent shell mass field; throws
  // std::invalid_argument if the field does not implement
  // ElasticParameterDependentMassField. Shape: (numVertices*3) x numParameterDofs.
  EigenSupport::SpMatD buildBodyForceParameterJacobian(
    const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
    const ShellMassField &massField) const;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
