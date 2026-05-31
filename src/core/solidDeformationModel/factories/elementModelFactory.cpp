#include "elementModelFactory.h"

#include "../formulations/elements/deformationGradientElementModel.h"
#include "../formulations/elements/shellElementModel.h"
#include "../simulationMesh.h"

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
  if (const auto *vol = dynamic_cast<const VolumetricFormulation *>(&formulation))
    return create_volume(mesh, ele, elasticBlock, plasticBlock, *vol);

  if (const auto *sh = dynamic_cast<const ShellFormulation *>(&formulation))
    return create_shell(mesh, ele, elasticBlock, plasticBlock, *sh);

  throw std::logic_error("ElementModelFactory: unsupported formulation type");
}

std::unique_ptr<DeformationModel> ElementModelFactory::create_volume(
  const SimulationMesh &mesh,
  int ele,
  const ElasticBlock &elasticBlock,
  const PlasticBlock &plasticBlock,
  const VolumetricFormulation &formulation)
{
  const int numNodes = formulation.getNodesPerElement();
  std::vector<double> restPosition(numNodes * 3);
  for (int j = 0; j < numNodes; j++) {
    ES::V3d p;
    mesh.getVertex(ele, j, p.data());
    restPosition[3 * j + 0] = p[0];
    restPosition[3 * j + 1] = p[1];
    restPosition[3 * j + 2] = p[2];
  }

  auto kernel = formulation.createKernel(restPosition.data());
  return std::make_unique<DeformationGradientElementModel>(
    ele, std::move(*kernel), elasticBlock, plasticBlock);
}

std::unique_ptr<DeformationModel> ElementModelFactory::create_shell(
  const SimulationMesh &mesh,
  int ele,
  const ElasticBlock &elasticBlock,
  const PlasticBlock &plasticBlock,
  const ShellFormulation &formulation)
{
  ES::V18d restPosition;
  bool hasVtx[6];
  for (int j = 0; j < 6; j++) {
    if (mesh.getVertexIndex(ele, j) < 0) {
      hasVtx[j] = false;
      restPosition.segment<3>(3 * j).setZero();
    }
    else {
      hasVtx[j] = true;
      ES::V3d p;
      mesh.getVertex(ele, j, p.data());
      restPosition.segment<3>(3 * j) = p;
    }
  }

  auto kernel = formulation.createKernel(restPosition.data(), hasVtx);
  return std::make_unique<ShellElementModel>(
    ele, std::move(kernel), elasticBlock, plasticBlock);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
