#pragma once

#include "../formulations/formulation.h"
#include "../formulations/elements/parameterizedMaterialBlock.h"
#include "../deformationModel.h"
#include "deformationModelManager.h"

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;

class ElementModelFactory
{
public:
  static std::unique_ptr<DeformationModel> create(
    const SimulationMesh &mesh,
    int ele,
    const ElasticBlock &elasticBlock,
    const PlasticBlock &plasticBlock,
    const Formulation &formulation);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
