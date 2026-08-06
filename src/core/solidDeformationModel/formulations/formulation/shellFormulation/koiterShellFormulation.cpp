#include "koiterShellFormulation.h"

#include "deformation/shell/koiterShellDeformationElement.h"
#include "formulations/formulation/formulationHelpers.h"
#include "simulation/simulationMesh.h"

#include "EigenSupport.h"

#include <array>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{
namespace
{
double triangleRestArea(const SimulationMesh &mesh, int ele)
{
  const ES::V3d &a = mesh.getVertex(ele, 0);
  const ES::V3d &b = mesh.getVertex(ele, 1);
  const ES::V3d &c = mesh.getVertex(ele, 2);
  return 0.5 * ((b - a).cross(c - a)).norm();
}
}  // namespace

std::string_view KoiterShellFormulation::getName() const { return "shell_koiter"; }
int KoiterShellFormulation::numBasisFunctionsPerElement() const { return 6; }
int KoiterShellFormulation::getLocalDofs() const { return 18; }

SimulationMeshType KoiterShellFormulation::compatibleMeshType() const
{
  return SimulationMeshType::SHELL;
}

std::unique_ptr<DeformationElement> KoiterShellFormulation::createElement(
  const SimulationMesh &mesh, int ele,
  std::unique_ptr<ElasticModel> elasticModel,
  std::unique_ptr<PlasticModel> plasticModel,
  DeformationElementConstructionOptions options) const
{
  ES::V18d restPosition = ES::V18d::Zero();
  std::array<bool, 6> hasVertex;
  for (int j = 0; j < 6; j++) {
    const int vertex = mesh.getVertexIndex(ele, j);
    hasVertex[j] = vertex >= 0;
    if (hasVertex[j])
      restPosition.segment<3>(3 * j) = mesh.getVertex(ele, j);
  }

  return std::make_unique<KoiterShellDeformationElement>(
    restPosition, hasVertex,
    detail::checkedMaterialCast<ElasticModel2DFundamentalForms>(
      std::move(elasticModel),
      "KoiterShellFormulation requires ElasticModel2DFundamentalForms."),
    detail::checkedMaterialCast<PlasticModel2DFundamentalForms>(
      std::move(plasticModel),
      "KoiterShellFormulation requires PlasticModel2DFundamentalForms."),
    options);
}

EigenSupport::SpMatD KoiterShellFormulation::buildMassMatrix(
  const SimulationMesh &mesh,
  EigenSupport::ConstRefVecXd elementArealDensities) const
{
  if (mesh.getElementType() != compatibleMeshType())
    throw std::invalid_argument("mesh type is incompatible with this formulation");

  detail::validateElementDensities(
    elementArealDensities, mesh.getNumElements(), "element areal density");

  std::vector<ES::TripletD> entries;
  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    const double mass =
      elementArealDensities[ele] * triangleRestArea(mesh, ele) / 3.0;
    for (int j = 0; j < 3; j++) {
      const int vertex = mesh.getVertexIndex(ele, j);
      for (int d = 0; d < 3; d++)
        entries.emplace_back(vertex * 3 + d, vertex * 3 + d, mass);
    }
  }

  const int numDofs = mesh.getNumVertices() * 3;
  ES::SpMatD massMatrix(numDofs, numDofs);
  massMatrix.setFromTriplets(entries.begin(), entries.end());
  return massMatrix;
}

EigenSupport::VXd KoiterShellFormulation::buildBodyForce(
  const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
  EigenSupport::ConstRefVecXd elementArealDensities) const
{
  if (mesh.getElementType() != compatibleMeshType())
    throw std::invalid_argument("mesh type is incompatible with this formulation");

  detail::validateElementDensities(
    elementArealDensities, mesh.getNumElements(), "element areal density");

  ES::VXd force = ES::VXd::Zero(mesh.getNumVertices() * 3);
  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    const double mass =
      elementArealDensities[ele] * triangleRestArea(mesh, ele) / 3.0;
    for (int j = 0; j < 3; j++) {
      const int vertex = mesh.getVertexIndex(ele, j);
      force.segment<3>(vertex * 3) += mass * acceleration;
    }
  }
  return force;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
