#include "volumetricFormulation.h"

#include "barycentricCoordinates.h"
#include "deformation/volume/volumetricDeformationElement.h"
#include "formulations/dof/dofLayout.h"
#include "formulations/formulation/formulationHelpers.h"
#include "formulations/quadrature/quadrature.h"
#include "formulations/shapeFunction/shapeFunction.h"
#include "simulation/simulationMesh.h"
#include "volumetricMesh.h"

#include <memory>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{
namespace
{
namespace ES = EigenSupport;

void localGlobalDofIndices(const DofLayout &layout, int ele, std::vector<int> &indices)
{
  indices.assign(layout.numLocalDofs(ele), -1);
  std::vector<DofGroup> groups;
  layout.getDofGroups(ele, groups);
  for (const DofGroup &group : groups)
    for (int i = 0; i < group.size; i++)
      indices[group.localStart + i] = group.globalDof(i);
}

std::vector<double> elementIntegrationWeights(
  std::span<const double> restPositions,
  const ShapeFunction &shapeFunction, const Quadrature &quadrature)
{
  const int numNodes = shapeFunction.numNodes();
  if (restPositions.size() != static_cast<std::size_t>(3 * numNodes))
    throw std::invalid_argument(
      "Element rest-position buffer has the wrong size.");

  Eigen::Matrix<double, 3, Eigen::Dynamic> restCoefficients(3, numNodes);
  for (int node = 0; node < numNodes; ++node) {
    restCoefficients.col(node) = ES::V3d(
      restPositions[node * 3], restPositions[node * 3 + 1],
      restPositions[node * 3 + 2]);
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> dN_dxi(3, numNodes);
  std::vector<double> result(quadrature.numPoints());
  for (int q = 0; q < quadrature.numPoints(); ++q) {
    const ES::V3d xi = quadrature.point(q);
    shapeFunction.compute_dN_dxi(xi[0], xi[1], xi[2], dN_dxi);
    result[q] = std::abs(
      (restCoefficients * dN_dxi.transpose()).determinant()) *
      quadrature.weight(q);
  }
  return result;
}
}  // namespace

VolumetricFormulation::VolumetricFormulation(
  std::unique_ptr<ShapeFunction> shapeFunction, std::unique_ptr<Quadrature> quadrature): shapeFunction_(std::move(shapeFunction)), quadrature_(std::move(quadrature))
{
}

VolumetricFormulation::~VolumetricFormulation() = default;

EigenSupport::SpMatD VolumetricFormulation::buildMassMatrix(
  const SimulationMesh &mesh,
  EigenSupport::ConstRefVecXd elementDensities) const
{
  if (mesh.getElementType() != compatibleMeshType()) {
    throw std::invalid_argument("mesh type is incompatible with this formulation");
  }
  detail::validateElementDensities(
    elementDensities, mesh.getNumElements(), "element density");

  const std::unique_ptr<DofLayout> dofLayout = createDofLayout(mesh);
  const ES::VXd restDofs = buildGlobalRestDofs(mesh);
  const ShapeFunction &sf = shapeFunction();
  const Quadrature &quad = massQuadrature();
  const int numNodes = sf.numNodes();

  ES::VXd N(numNodes);
  std::vector<double> localRest;
  std::vector<int> globalIdx;
  std::vector<ES::TripletD> entries;
  std::vector<DofGroup> groups;

  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    localRest.resize(dofLayout->numLocalDofs(ele));
    dofLayout->gather(
      ele,
      std::span<const double>(restDofs.data(), static_cast<std::size_t>(restDofs.size())),
      localRest,
      groups);
    const std::vector<double> integrationWeights =
      elementIntegrationWeights(localRest, sf, quad);
    localGlobalDofIndices(*dofLayout, ele, globalIdx);
    const double rho = elementDensities[ele];

    for (int q = 0; q < quad.numPoints(); q++) {
      const ES::V3d xi = quad.point(q);
      sf.compute_N(xi[0], xi[1], xi[2], N);
      const double w = rho * integrationWeights[q];

      for (int a = 0; a < numNodes; a++) {
        const double wa = w * N[a];
        if (wa == 0.0)
          continue;
        for (int b = 0; b < numNodes; b++) {
          const double m = wa * N[b];
          if (m == 0.0)
            continue;
          for (int d = 0; d < 3; d++) {
            const int ga = globalIdx[a * 3 + d];
            const int gb = globalIdx[b * 3 + d];
            if (ga < 0 || gb < 0)
              continue;
            entries.emplace_back(ga, gb, m);
          }
        }
      }
    }
  }

  ES::SpMatD M(dofLayout->numGlobalDofs(), dofLayout->numGlobalDofs());
  M.setFromTriplets(entries.begin(), entries.end());
  return M;
}

EigenSupport::VXd VolumetricFormulation::buildBodyForce(
  const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
  EigenSupport::ConstRefVecXd elementDensities) const
{
  if (mesh.getElementType() != compatibleMeshType()) {
    throw std::invalid_argument("mesh type is incompatible with this formulation");
  }
  detail::validateElementDensities(
    elementDensities, mesh.getNumElements(), "element density");

  const std::unique_ptr<DofLayout> dofLayout = createDofLayout(mesh);
  const ES::VXd restDofs = buildGlobalRestDofs(mesh);
  const ShapeFunction &sf = shapeFunction();
  const Quadrature &quad = massQuadrature();
  const int numNodes = sf.numNodes();

  ES::VXd N(numNodes);
  std::vector<double> localRest;
  std::vector<int> globalIdx;
  std::vector<DofGroup> groups;
  ES::VXd f = ES::VXd::Zero(dofLayout->numGlobalDofs());

  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    localRest.resize(dofLayout->numLocalDofs(ele));
    dofLayout->gather(
      ele,
      std::span<const double>(restDofs.data(), static_cast<std::size_t>(restDofs.size())),
      localRest,
      groups);
    const std::vector<double> integrationWeights =
      elementIntegrationWeights(localRest, sf, quad);
    localGlobalDofIndices(*dofLayout, ele, globalIdx);
    const double rho = elementDensities[ele];

    for (int q = 0; q < quad.numPoints(); q++) {
      const ES::V3d xi = quad.point(q);
      sf.compute_N(xi[0], xi[1], xi[2], N);
      const double w = rho * integrationWeights[q];

      for (int a = 0; a < numNodes; a++) {
        const double fa = w * N[a];
        if (fa == 0.0)
          continue;
        for (int d = 0; d < 3; d++) {
          const int ga = globalIdx[a * 3 + d];
          if (ga < 0)
            continue;
          f[ga] += fa * acceleration[d];
        }
      }
    }
  }

  return f;
}

EigenSupport::SpMatD VolumetricFormulation::buildSurfaceEmbeddingMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const EigenSupport::MXd &surfaceVertices) const
{
  const int numTargets = static_cast<int>(surfaceVertices.rows());
  const std::vector<double> flat =
    detail::flattenSurfaceVertices(surfaceVertices);
  InterpolationCoordinates::BarycentricCoordinates bc(numTargets, flat.data(), &mesh);
  return bc.generateInterpolationMatrix();
}

std::unique_ptr<DeformationElement> VolumetricFormulation::createElement(
  const SimulationMesh &mesh, int ele,
  std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel,
  DeformationElementConstructionOptions options) const
{
  const int numNodes = shapeFunction_->numNodes();
  std::vector<double> restPosition(numNodes * 3);
  for (int j = 0; j < numNodes; j++) {
    const EigenSupport::V3d &vertex = mesh.getVertex(ele, j);
    restPosition[3 * j + 0] = vertex[0];
    restPosition[3 * j + 1] = vertex[1];
    restPosition[3 * j + 2] = vertex[2];
  }

  return std::make_unique<VolumetricDeformationElement>(
    restPosition, *shapeFunction_, *quadrature_,
    detail::checkedMaterialCast<ElasticModel3DDeformationGradient>(
      std::move(elasticModel),
      "VolumetricFormulation requires ElasticModel3DDeformationGradient."),
    detail::checkedMaterialCast<PlasticModel3DDeformationGradient>(
      std::move(plasticModel),
      "VolumetricFormulation requires PlasticModel3DDeformationGradient."),
    options);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
