#include "elementModelFactory.h"

namespace pgo
{
namespace SolidDeformationModel
{

std::unique_ptr<DeformationModel> ElementModelFactory::create(
  const SimulationMesh &mesh,
  int ele,
  const ElasticBlock &elasticBlock,
  const PlasticBlock &plasticBlock,
  const Formulation &formulation)
{
  return formulation.createElement(mesh, ele, elasticBlock, plasticBlock);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
