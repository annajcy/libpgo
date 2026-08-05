#include "shellFormulation.h"

#include "deformation/shell/shellDeformationElement.h"
#include "simulation/simulationMesh.h"

#include "EigenSupport.h"

#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{
namespace
{
namespace ES = EigenSupport;

void validateElementArealDensities(
  ES::ConstRefVecXd densities, int numElements)
{
  if (densities.size() != numElements)
    throw std::invalid_argument(
      "element areal density count does not match mesh element count");
  if (!densities.allFinite() || (densities.array() <= 0.0).any())
    throw std::invalid_argument(
      "element areal densities must contain finite values > 0");
}

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

double triangleRestArea(const SimulationMesh &mesh, int ele)
{
  const ES::V3d &a = mesh.getVertex(ele, 0);
  const ES::V3d &b = mesh.getVertex(ele, 1);
  const ES::V3d &c = mesh.getVertex(ele, 2);
  return 0.5 * ((b - a).cross(c - a)).norm();
}
}  // namespace

SimulationMeshType ShellFormulation::compatibleMeshType() const
{
  return SimulationMeshType::SHELL;
}

std::unique_ptr<DeformationElement> ShellFormulation::createElement(
  const SimulationMesh &mesh, int ele,
  std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel,
  DeformationElementConstructionOptions options) const
{
  ES::V18d restPosition = ES::V18d::Zero();
  std::array<bool, 6> hasVtx;
  for (int j = 0; j < 6; j++) {
    if (mesh.getVertexIndex(ele, j) < 0) {
      hasVtx[j] = false;
    }
    else {
      hasVtx[j] = true;
      restPosition.segment<3>(3 * j) = mesh.getVertex(ele, j);
    }
  }

  auto mapping = createElementMapping(restPosition, hasVtx);
  return std::make_unique<ShellDeformationElement>(
    std::move(mapping),
    checkedMaterialCast<ElasticModel2DFundamentalForms>(
      std::move(elasticModel),
      "ShellFormulation requires ElasticModel2DFundamentalForms."),
    checkedMaterialCast<PlasticModel2DFundamentalForms>(
      std::move(plasticModel),
      "ShellFormulation requires PlasticModel2DFundamentalForms."),
    options);
}

EigenSupport::SpMatD ShellFormulation::buildMassMatrix(
  const SimulationMesh &mesh,
  EigenSupport::ConstRefVecXd elementArealDensities) const
{
  if (mesh.getElementType() != compatibleMeshType()) {
    throw std::invalid_argument("mesh type is incompatible with this formulation");
  }

  validateElementArealDensities(
    elementArealDensities, mesh.getNumElements());

  std::vector<ES::TripletD> entries;
  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    const double m = elementArealDensities[ele] * triangleRestArea(mesh, ele) / 3.0;
    for (int j = 0; j < 3; j++) {
      const int v = mesh.getVertexIndex(ele, j);
      for (int d = 0; d < 3; d++)
        entries.emplace_back(v * 3 + d, v * 3 + d, m);
    }
  }

  const int numDofs = mesh.getNumVertices() * 3;
  ES::SpMatD M(numDofs, numDofs);
  M.setFromTriplets(entries.begin(), entries.end());
  return M;
}

EigenSupport::VXd ShellFormulation::buildBodyForce(
  const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
  EigenSupport::ConstRefVecXd elementArealDensities) const
{
  if (mesh.getElementType() != compatibleMeshType()) {
    throw std::invalid_argument("mesh type is incompatible with this formulation");
  }

  validateElementArealDensities(
    elementArealDensities, mesh.getNumElements());

  ES::VXd f = ES::VXd::Zero(mesh.getNumVertices() * 3);
  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    const double m = elementArealDensities[ele] * triangleRestArea(mesh, ele) / 3.0;
    for (int j = 0; j < 3; j++) {
      const int v = mesh.getVertexIndex(ele, j);
      f.segment<3>(v * 3) += m * acceleration;
    }
  }
  return f;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
