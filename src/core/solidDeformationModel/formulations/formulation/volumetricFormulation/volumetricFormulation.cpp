#include "volumetricFormulation.h"

#include "barycentricCoordinates.h"
#include "deformation/volume/volumetricDeformationModel.h"
#include "deformation/volume/volumetricElementMapping.h"
#include "formulations/quadrature/quadrature.h"
#include "formulations/shapeFunction/shapeFunction.h"
#include "generateMassMatrix.h"
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
  const double *restPositions) const
{
  return std::make_unique<VolumetricElementMapping>(
    restPositions, shapeFunction_->clone(), quadrature_->clone());
}

EigenSupport::SpMatD VolumetricFormulation::buildMassMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh) const
{
  ES::SpMatD M;
  VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(&mesh, M, true);
  return M;
}

EigenSupport::VXd VolumetricFormulation::buildBodyForce(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const EigenSupport::V3d &acceleration) const
{
  ES::SpMatD M;
  VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(&mesh, M, true);
  ES::VXd accelField(mesh.getNumVertices() * 3);
  for (int vertex = 0; vertex < mesh.getNumVertices(); vertex++)
    accelField.segment<3>(vertex * 3) = acceleration;
  return M * accelField;
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
  std::unique_ptr<ElasticModel> elasticModel, std::unique_ptr<PlasticModel> plasticModel) const
{
  const int numNodes = getNodesPerElement();
  std::vector<double> restPosition(numNodes * 3);
  for (int j = 0; j < numNodes; j++)
    mesh.getVertex(ele, j, &restPosition[3 * j]);

  auto mapping = createElementMapping(restPosition.data());
  return std::make_unique<VolumetricDeformationModel>(
    std::move(*mapping),
    checkedMaterialCast<ElasticModel3DDeformationGradient>(
      std::move(elasticModel),
      "VolumetricFormulation requires ElasticModel3DDeformationGradient."),
    checkedMaterialCast<PlasticModel3DDeformationGradient>(
      std::move(plasticModel),
      "VolumetricFormulation requires PlasticModel3DDeformationGradient."));
}

}  // namespace SolidDeformationModel
}  // namespace pgo
