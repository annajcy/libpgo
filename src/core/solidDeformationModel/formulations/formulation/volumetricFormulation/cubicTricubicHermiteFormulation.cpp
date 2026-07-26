#include "cubicTricubicHermiteFormulation.h"

#include "barycentricCoordinates.h"
#include "deformation/volume/volumetricDeformationModel.h"
#include "formulations/dof/cubicTricubicHermiteDofLayout.h"
#include "formulations/quadrature/gaussLegendreHexQuadrature.h"
#include "formulations/shapeFunction/cubicTricubicHermiteShapeFunction.h"
#include "simulation/simulationMesh.h"
#include "volumetricMesh.h"

#include <array>
#include <memory>
#include <stdexcept>
#include <vector>

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

namespace ES = EigenSupport;

constexpr int kHermiteNodes = 64;
constexpr int kHermiteModes = 8;

void elementHermiteRestDofs(const SimulationMesh &mesh, int ele, std::array<double, 192> &rest)
{
  std::array<ES::V3d, 8> P;
  for (int c = 0; c < 8; c++)
    P[c] = mesh.getVertex(ele, c);

  const ES::V3d dXi = P[1] - P[0];
  const ES::V3d dEta = P[3] - P[0];
  const ES::V3d dZeta = P[4] - P[0];

  rest.fill(0.0);
  for (int c = 0; c < 8; c++) {
    const ES::V3d *mode[4] = { &P[c], &dXi, &dEta, &dZeta };
    for (int m = 0; m < 4; m++) {
      const int node = c * 8 + m;
      for (int k = 0; k < 3; k++)
        rest[node * 3 + k] = (*mode[m])[k];
    }
  }
}

std::vector<double> flattenSurfaceVertices(const ES::MXd &surfaceVertices)
{
  if (surfaceVertices.cols() != 3) {
    throw std::invalid_argument("surfaceVertices must have shape numVertices x 3");
  }
  std::vector<double> flat(static_cast<size_t>(surfaceVertices.rows()) * 3);
  for (Eigen::Index i = 0; i < surfaceVertices.rows(); i++)
    for (int d = 0; d < 3; d++)
      flat[static_cast<size_t>(i) * 3 + d] = surfaceVertices(i, d);
  return flat;
}

ES::SpMatD buildHermiteSurfaceEmbeddingMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const ES::MXd &surfaceVertices)
{
  if (mesh.getNumElementVertices() != 8) {
    throw std::invalid_argument("cubic_tricubic_hermite surface embedding requires an 8-corner cubic mesh");
  }

  const int numTargets = static_cast<int>(surfaceVertices.rows());
  const std::vector<double> flat = flattenSurfaceVertices(surfaceVertices);
  InterpolationCoordinates::BarycentricCoordinates bc(numTargets, flat.data(), &mesh);

  if (bc.getNumElementVertices() != 8) {
    throw std::runtime_error("Hermite surface embedding expected 8 interpolation weights per target");
  }

  CubicTricubicHermiteShapeFunction shapeFunction;
  std::vector<ES::TripletD> entries;
  entries.reserve(static_cast<size_t>(numTargets) * kHermiteNodes * 3);

  for (int target = 0; target < numTargets; target++) {
    const ES::V8d weights = Eigen::Map<const ES::V8d>(
      bc.getEmbeddingWeights(target));
    const int *indices = bc.getEmbeddingVertexIndices(target);

    const ES::V3d q = CubicFormulation::clampedParametricCoordinates(weights);
    const ES::V64d H = shapeFunction.compute_N(q[0], q[1], q[2]);

    for (int node = 0; node < kHermiteNodes; node++) {
      const double value = H[node];
      if (value == 0.0)
        continue;
      const int vertex = indices[node / kHermiteModes];
      const int mode = node % kHermiteModes;
      for (int coord = 0; coord < 3; coord++) {
        entries.emplace_back(
          target * 3 + coord,
          vertex * CubicTricubicHermiteDofLayout::kDofsPerVertex + mode * 3 + coord,
          value);
      }
    }
  }

  ES::SpMatD W(numTargets * 3, mesh.getNumVertices() * CubicTricubicHermiteDofLayout::kDofsPerVertex);
  W.setFromTriplets(entries.begin(), entries.end());
  return W;
}
}  // namespace

CubicTricubicHermiteFormulation::CubicTricubicHermiteFormulation()
  : CubicFormulation(
      std::make_unique<CubicTricubicHermiteShapeFunction>(),
      std::make_unique<GaussLegendreHexQuadrature4>())
{
}

std::string_view CubicTricubicHermiteFormulation::getName() const { return "cubic_tricubic_hermite"; }
int CubicTricubicHermiteFormulation::numBasisFunctionsPerElement() const { return 64; }
int CubicTricubicHermiteFormulation::getLocalDofs() const { return 192; }

EigenSupport::SpMatD CubicTricubicHermiteFormulation::buildSurfaceEmbeddingMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const EigenSupport::MXd &surfaceVertices) const
{
  return buildHermiteSurfaceEmbeddingMatrix(mesh, surfaceVertices);
}

std::unique_ptr<DofLayout> CubicTricubicHermiteFormulation::createDofLayout(const SimulationMesh &mesh) const
{
  return std::make_unique<CubicTricubicHermiteDofLayout>(mesh);
}

EigenSupport::VXd CubicTricubicHermiteFormulation::buildGlobalRestDofs(const SimulationMesh &mesh) const
{
  const int nvtx = mesh.getNumVertices();
  EigenSupport::VXd rest = EigenSupport::VXd::Zero(nvtx * CubicTricubicHermiteDofLayout::kDofsPerVertex);

  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    std::array<double, 192> local;
    elementHermiteRestDofs(mesh, ele, local);
    for (int c = 0; c < 8; c++) {
      int v = mesh.getVertexIndex(ele, c);
      if (v < 0)
        continue;
      const int base = v * CubicTricubicHermiteDofLayout::kDofsPerVertex;
      for (int dof = 0; dof < CubicTricubicHermiteDofLayout::kDofsPerVertex; dof++)
        rest[base + dof] = local[c * CubicTricubicHermiteDofLayout::kDofsPerVertex + dof];
    }
  }
  return rest;
}

std::unique_ptr<DeformationModel> CubicTricubicHermiteFormulation::createElement(
  const SimulationMesh &mesh, int ele,
  std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel,
  DeformationModelConstructionOptions options) const
{
  std::array<double, 192> restPosition;
  elementHermiteRestDofs(mesh, ele, restPosition);

  auto mapping = createElementMapping(restPosition);
  return std::make_unique<VolumetricDeformationModel>(
    std::move(*mapping),
    checkedMaterialCast<ElasticModel3DDeformationGradient>(
      std::move(elasticModel),
      "CubicTricubicHermiteFormulation requires ElasticModel3DDeformationGradient."),
    checkedMaterialCast<PlasticModel3DDeformationGradient>(
      std::move(plasticModel),
      "CubicTricubicHermiteFormulation requires PlasticModel3DDeformationGradient."),
    options);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
