#include "volumetricFormulation.h"

#include "barycentricCoordinates.h"
#include "deformation/volume/volumetricDeformationModel.h"
#include "deformation/volume/volumetricElementMapping.h"
#include "formulations/dof/dofLayout.h"
#include "formulations/quadrature/quadrature.h"
#include "formulations/shapeFunction/shapeFunction.h"
#include "mass/volumeDensityField.h"
#include "simulation/simulationMesh.h"
#include "volumetricMesh.h"

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

void localGlobalDofIndices(const DofLayout &layout, int ele, std::vector<int> &indices)
{
  indices.assign(layout.numLocalDofs(ele), -1);
  std::vector<DofGroup> groups;
  layout.getDofGroups(ele, groups);
  for (const DofGroup &group : groups)
    for (int i = 0; i < group.size; i++)
      indices[group.localStart + i] = group.globalDof(i);
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
}  // namespace

VolumetricFormulation::VolumetricFormulation(
  std::unique_ptr<ShapeFunction> shapeFunction, std::unique_ptr<Quadrature> quadrature)
  : shapeFunction_(std::move(shapeFunction)), quadrature_(std::move(quadrature))
{
}

VolumetricFormulation::~VolumetricFormulation() = default;

std::unique_ptr<VolumetricElementMapping> VolumetricFormulation::createElementMapping(
  std::span<const double> restPositions) const
{
  return std::make_unique<VolumetricElementMapping>(
    restPositions, shapeFunction_->clone(), quadrature_->clone());
}

EigenSupport::SpMatD VolumetricFormulation::buildMassMatrix(
  const SimulationMesh &mesh, const VolumeDensityField &density) const
{
  if (mesh.getElementType() != compatibleMeshType()) {
    throw std::invalid_argument("mesh type is incompatible with this formulation");
  }
  density.validate(mesh.getNumElements());

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
    const VolumetricElementMapping mapping(localRest, sf, quad);
    localGlobalDofIndices(*dofLayout, ele, globalIdx);
    const double rho = density.value(ele);

    for (int q = 0; q < quad.numPoints(); q++) {
      const ES::V3d xi = quad.point(q);
      sf.compute_N(xi[0], xi[1], xi[2], N);
      const double w = rho * mapping.weightDetJ(q);

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
  const VolumeDensityField &density) const
{
  if (mesh.getElementType() != compatibleMeshType()) {
    throw std::invalid_argument("mesh type is incompatible with this formulation");
  }
  density.validate(mesh.getNumElements());

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
    const VolumetricElementMapping mapping(localRest, sf, quad);
    localGlobalDofIndices(*dofLayout, ele, globalIdx);
    const double rho = density.value(ele);

    for (int q = 0; q < quad.numPoints(); q++) {
      const ES::V3d xi = quad.point(q);
      sf.compute_N(xi[0], xi[1], xi[2], N);
      const double w = rho * mapping.weightDetJ(q);

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
  const std::vector<double> flat = flattenSurfaceVertices(surfaceVertices);
  InterpolationCoordinates::BarycentricCoordinates bc(numTargets, flat.data(), &mesh);
  return bc.generateInterpolationMatrix();
}

std::unique_ptr<DeformationModel> VolumetricFormulation::createElement(
  const SimulationMesh &mesh, int ele,
  std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel,
  DeformationModelConstructionOptions options) const
{
  const int numNodes = shapeFunction_->numNodes();
  std::vector<double> restPosition(numNodes * 3);
  for (int j = 0; j < numNodes; j++) {
    const EigenSupport::V3d &vertex = mesh.getVertex(ele, j);
    restPosition[3 * j + 0] = vertex[0];
    restPosition[3 * j + 1] = vertex[1];
    restPosition[3 * j + 2] = vertex[2];
  }

  auto mapping = createElementMapping(restPosition);
  return std::make_unique<VolumetricDeformationModel>(
    std::move(*mapping),
    checkedMaterialCast<ElasticModel3DDeformationGradient>(
      std::move(elasticModel),
      "VolumetricFormulation requires ElasticModel3DDeformationGradient."),
    checkedMaterialCast<PlasticModel3DDeformationGradient>(
      std::move(plasticModel),
      "VolumetricFormulation requires PlasticModel3DDeformationGradient."),
    options);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
