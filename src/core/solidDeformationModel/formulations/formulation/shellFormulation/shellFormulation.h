#pragma once

#include "formulations/formulation/formulation.h"

namespace pgo
{
namespace SolidDeformationModel
{

class ShellElementMapping;

class ShellFormulation : public Formulation
{
public:
  virtual std::unique_ptr<ShellElementMapping> createElementMapping(
    const double restX[18], const bool hasVtx[6]) const = 0;

  std::unique_ptr<DeformationModel> createElement(
    const SimulationMesh &mesh, int ele,
    std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel) const override;

  SimulationMeshType compatibleMeshType() const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
