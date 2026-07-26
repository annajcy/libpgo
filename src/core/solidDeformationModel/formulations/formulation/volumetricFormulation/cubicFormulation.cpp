#include "cubicFormulation.h"

#include "barycentricCoordinates.h"
#include "simulation/simulationMesh.h"
#include "volumetricMesh.h"

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

SimulationMeshType CubicFormulation::compatibleMeshType() const
{
  return SimulationMeshType::CUBIC;
}

EigenSupport::V3d CubicFormulation::clampedParametricCoordinates(
  const EigenSupport::V8d &w)
{
  return EigenSupport::V3d(
    std::clamp(w[1] + w[2] + w[5] + w[6], 0.0, 1.0),
    std::clamp(w[2] + w[3] + w[6] + w[7], 0.0, 1.0),
    std::clamp(w[4] + w[5] + w[6] + w[7], 0.0, 1.0));
}

EigenSupport::SpMatD CubicFormulation::buildSurfaceEmbeddingMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const EigenSupport::MXd &surfaceVertices) const
{
  if (surfaceVertices.cols() != 3)
    throw std::invalid_argument("surfaceVertices must have shape numVertices x 3");

  std::vector<double> points(static_cast<size_t>(surfaceVertices.size()));
  for (Eigen::Index i = 0; i < surfaceVertices.rows(); i++)
    for (int d = 0; d < 3; d++)
      points[static_cast<size_t>(i) * 3 + d] = surfaceVertices(i, d);

  InterpolationCoordinates::BarycentricCoordinates bc(
    static_cast<int>(surfaceVertices.rows()), points.data(), &mesh);
  std::vector<EigenSupport::TripletD> entries;
  entries.reserve(static_cast<size_t>(surfaceVertices.rows()) * 8 * 3);

  for (int target = 0; target < surfaceVertices.rows(); target++) {
    const int *vertices = bc.getEmbeddingVertexIndices(target);
    const EigenSupport::V8d paramWeights = Eigen::Map<const EigenSupport::V8d>(
      bc.getEmbeddingWeights(target));
    const EigenSupport::V3d q = clampedParametricCoordinates(paramWeights);
    const EigenSupport::V2d x(1.0 - q[0], q[0]);
    const EigenSupport::V2d y(1.0 - q[1], q[1]);
    const EigenSupport::V2d z(1.0 - q[2], q[2]);
    const EigenSupport::V8d weights = (EigenSupport::V8d() <<
      x[0] * y[0] * z[0], x[1] * y[0] * z[0], x[1] * y[1] * z[0], x[0] * y[1] * z[0],
      x[0] * y[0] * z[1], x[1] * y[0] * z[1], x[1] * y[1] * z[1], x[0] * y[1] * z[1]).finished();
    for (int corner = 0; corner < 8; corner++)
      for (int d = 0; d < 3; d++)
        entries.emplace_back(target * 3 + d, vertices[corner] * 3 + d, weights[corner]);
  }

  EigenSupport::SpMatD W(surfaceVertices.rows() * 3, mesh.getNumVertices() * 3);
  W.setFromTriplets(entries.begin(), entries.end());
  return W;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
