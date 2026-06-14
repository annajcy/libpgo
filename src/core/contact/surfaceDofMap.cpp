#include "surfaceDofMap.h"

#include <numeric>
#include <stdexcept>

namespace pgo
{
namespace Contact
{

SurfaceDofMap::SurfaceDofMap(
  const EigenSupport::MXd &surfaceRestVertices,
  const EigenSupport::SpMatD &surfaceFromSimulationDispMap):
  surfaceFromSimulationDispMap_(surfaceFromSimulationDispMap)
{
  if (surfaceRestVertices.cols() != 3)
    throw std::invalid_argument("SurfaceDofMap rest vertices must have shape (#vertices, 3).");
  if (surfaceRestVertices.rows() <= 0)
    throw std::invalid_argument("SurfaceDofMap rest vertices must contain at least one vertex.");
  if (surfaceFromSimulationDispMap_.rows() != surfaceRestVertices.rows() * 3)
    throw std::invalid_argument("SurfaceDofMap surface-from-simulation map row count must be 3 * #surface vertices.");
  if (surfaceFromSimulationDispMap_.cols() <= 0)
    throw std::invalid_argument("SurfaceDofMap surface-from-simulation map must contain at least one simulation DOF.");

  surfaceRestPositions_.resize(surfaceRestVertices.rows() * 3);
  for (Eigen::Index i = 0; i < surfaceRestVertices.rows(); i++)
    surfaceRestPositions_.segment<3>(i * 3) = surfaceRestVertices.row(i).transpose();

  simulationDofs_.resize(surfaceFromSimulationDispMap_.cols());
  std::iota(simulationDofs_.begin(), simulationDofs_.end(), 0);
}

void SurfaceDofMap::validateSimulationDisplacementSize(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  if (simulationDisplacements.size() != surfaceFromSimulationDispMap_.cols())
    throw std::invalid_argument("Simulation displacement size does not match SurfaceDofMap column count.");
}

void SurfaceDofMap::validateSurfaceVectorSize(EigenSupport::ConstRefVecXd surfaceVector) const
{
  if (surfaceVector.size() != surfaceFromSimulationDispMap_.rows())
    throw std::invalid_argument("Surface vector size does not match SurfaceDofMap row count.");
}

EigenSupport::VXd SurfaceDofMap::surfaceDisplacements(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  validateSimulationDisplacementSize(simulationDisplacements);
  return surfaceFromSimulationDispMap_ * simulationDisplacements;
}

EigenSupport::VXd SurfaceDofMap::surfacePositions(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  return surfaceRestPositions_ + surfaceDisplacements(simulationDisplacements);
}

EigenSupport::VXd SurfaceDofMap::pullbackGradient(EigenSupport::ConstRefVecXd surfaceGradient) const
{
  validateSurfaceVectorSize(surfaceGradient);
  return surfaceFromSimulationDispMap_.transpose() * surfaceGradient;
}

void SurfaceDofMap::pullbackHessian(const EigenSupport::SpMatD &surfaceHessian, EigenSupport::SpMatD &simulationHessian) const
{
  if (surfaceHessian.rows() != surfaceFromSimulationDispMap_.rows() ||
    surfaceHessian.cols() != surfaceFromSimulationDispMap_.rows())
    throw std::invalid_argument("Surface Hessian shape does not match SurfaceDofMap row count.");

  simulationHessian = surfaceFromSimulationDispMap_.transpose() * surfaceHessian * surfaceFromSimulationDispMap_;
}

}  // namespace Contact
}  // namespace pgo
