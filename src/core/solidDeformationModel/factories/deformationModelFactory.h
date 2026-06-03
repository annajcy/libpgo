#pragma once

#include "../formulations/formulation.h"
#include "../deformationModel.h"

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;

class DeformationModelFactory
{
public:
  static std::unique_ptr<DeformationModel> create(
    const SimulationMesh &mesh,
    int ele,
    std::unique_ptr<ElasticModel> elasticModel,
    std::unique_ptr<PlasticModel> plasticModel,
    const ParameterField *elasticParams,
    const ParameterField *plasticParams,
    const Formulation &formulation);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
