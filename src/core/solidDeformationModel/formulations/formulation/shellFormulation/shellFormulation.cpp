#include "shellFormulation.h"

#include "deformation/shell/shellDeformationModel.h"
#include "simulation/simulationMesh.h"

#include <memory>
#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{
namespace
{
template<class Derived, class Base>
std::unique_ptr<Derived> checkedMaterialCast(
  std::unique_ptr<Base> model, const char *message)
{
  if (Derived *typed = dynamic_cast<Derived *>(model.get())) {
    model.release();
    return std::unique_ptr<Derived>(typed);
  }

  throw std::invalid_argument(message);
}
}  // namespace

SimulationMeshType ShellFormulation::compatibleMeshType() const
{
  return SimulationMeshType::SHELL;
}

std::unique_ptr<DeformationModel> ShellFormulation::createElement(
  const SimulationMesh &mesh, int ele,
  std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel) const
{
  double restPosition[18] = {};
  bool hasVtx[6];
  for (int j = 0; j < 6; j++) {
    if (mesh.getVertexIndex(ele, j) < 0) {
      hasVtx[j] = false;
    }
    else {
      hasVtx[j] = true;
      mesh.getVertex(ele, j, restPosition + 3 * j);
    }
  }

  auto mapping = createElementMapping(restPosition, hasVtx);
  return std::make_unique<ShellDeformationModel>(
    std::move(mapping),
    checkedMaterialCast<ElasticModel2DFundamentalForms>(
      std::move(elasticModel),
      "ShellFormulation requires ElasticModel2DFundamentalForms."),
    checkedMaterialCast<PlasticModel2DFundamentalForms>(
      std::move(plasticModel),
      "ShellFormulation requires PlasticModel2DFundamentalForms."));
}

}  // namespace SolidDeformationModel
}  // namespace pgo
