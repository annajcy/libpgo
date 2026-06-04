#include "embeddedDofMap.h"

#include <numeric>
#include <stdexcept>

namespace pgo
{
namespace Contact
{

EmbeddedDofMap::EmbeddedDofMap(
  const EigenSupport::MXd &surfaceRestVertices,
  const EigenSupport::SpMatD &surfaceFromSimulationDispMap):
  surfaceFromSimulationDispMap_(surfaceFromSimulationDispMap)
{
  if (surfaceRestVertices.cols() != 3)
    throw std::invalid_argument("ContactSurfaceSpec.restVertices must have shape (#vertices, 3).");
  if (surfaceFromSimulationDispMap_.rows() != surfaceRestVertices.rows() * 3)
    throw std::invalid_argument("ContactSurfaceSpec.surfaceFromSimulationDispMap row count must be 3 * #surface vertices.");

  surfaceRestPositions_.resize(surfaceRestVertices.rows() * 3);
  for (Eigen::Index i = 0; i < surfaceRestVertices.rows(); i++)
    surfaceRestPositions_.segment<3>(i * 3) = surfaceRestVertices.row(i).transpose();

  simulationDofs_.resize(surfaceFromSimulationDispMap_.cols());
  std::iota(simulationDofs_.begin(), simulationDofs_.end(), 0);
}

void EmbeddedDofMap::validateSimulationDisplacementSize(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  if (simulationDisplacements.size() != surfaceFromSimulationDispMap_.cols())
    throw std::invalid_argument("Simulation displacement size does not match embedded surface map column count.");
}

EigenSupport::VXd EmbeddedDofMap::surfaceDisplacements(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  validateSimulationDisplacementSize(simulationDisplacements);
  return surfaceFromSimulationDispMap_ * simulationDisplacements;
}

EigenSupport::VXd EmbeddedDofMap::surfacePositions(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  return surfaceRestPositions_ + surfaceDisplacements(simulationDisplacements);
}

}  // namespace Contact
}  // namespace pgo
