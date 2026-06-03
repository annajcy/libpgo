#include "deformationModelFactory.h"

namespace pgo
{
namespace SolidDeformationModel
{

std::unique_ptr<DeformationModel> DeformationModelFactory::create(
  const SimulationMesh &mesh,
  int ele,
  std::unique_ptr<ElasticModel> elasticModel,
  std::unique_ptr<PlasticModel> plasticModel,
  const ParameterField *elasticParams,
  const ParameterField *plasticParams,
  const Formulation &formulation)
{
  return formulation.createElement(mesh, ele,
    std::move(elasticModel), std::move(plasticModel),
    elasticParams, plasticParams);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
