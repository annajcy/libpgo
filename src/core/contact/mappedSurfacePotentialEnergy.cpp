/*
copyright to Bohan Wang
*/

#include "mappedSurfacePotentialEnergy.h"

#include "scopedProfileSection.h"
#include "ipc/profiling/surfaceIPCProfiling.h"

#include <numeric>
#include <stdexcept>

namespace pgo
{
namespace Contact
{
namespace IPC
{

MappedSurfacePotentialEnergy::MappedSurfacePotentialEnergy(
  const EigenSupport::MXd &surfaceRestVertices,
  const EigenSupport::SpMatD &surfaceFromSimulationDispMap):
  surfaceFromSimulationDispMap_(surfaceFromSimulationDispMap)
{
  if (surfaceRestVertices.cols() != 3)
    throw std::invalid_argument("surfaceRestVertices must be an N x 3 matrix.");
  if (surfaceRestVertices.rows() <= 0)
    throw std::invalid_argument("surfaceRestVertices must contain at least one vertex.");
  if (surfaceFromSimulationDispMap_.rows() != surfaceRestVertices.rows() * 3)
    throw std::invalid_argument("surfaceFromSimulationDispMap row count must equal 3 * numSurfaceVertices.");
  if (surfaceFromSimulationDispMap_.cols() <= 0)
    throw std::invalid_argument("surfaceFromSimulationDispMap must contain at least one simulation DOF.");

  surfaceRestPositions_.resize(surfaceRestVertices.rows() * 3);
  for (int vi = 0; vi < surfaceRestVertices.rows(); ++vi)
    surfaceRestPositions_.segment<3>(3 * vi) = surfaceRestVertices.row(vi).transpose();

  simulationDOFs_.resize(static_cast<std::size_t>(surfaceFromSimulationDispMap_.cols()));
  std::iota(simulationDOFs_.begin(), simulationDOFs_.end(), 0);
}

void MappedSurfacePotentialEnergy::validateSimulationDisplacementSize(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  if (simulationDisplacements.size() != surfaceFromSimulationDispMap_.cols())
    throw std::invalid_argument("Simulation displacement vector size does not match surfaceFromSimulationDispMap column count.");
}

VXd MappedSurfacePotentialEnergy::computeSurfaceDisplacementsFromSimulationDisplacements(
  EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  validateSimulationDisplacementSize(simulationDisplacements);
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterMapToSurface);
  return VXd(surfaceFromSimulationDispMap_ * simulationDisplacements);
}

VXd MappedSurfacePotentialEnergy::computeSurfacePositionsFromSimulationDisplacements(
  EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  return surfaceRestPositions_ + computeSurfaceDisplacementsFromSimulationDisplacements(simulationDisplacements);
}

VXd MappedSurfacePotentialEnergy::pullbackSurfaceGradient(EigenSupport::ConstRefVecXd surfaceGradient) const
{
  Profiling::ScopedProfileSection pullbackProfile(SurfaceIPCProfileSections::kAdapterPullbackGradient);
  return surfaceFromSimulationDispMap_.transpose() * surfaceGradient;
}

void MappedSurfacePotentialEnergy::pullbackSurfaceHessian(
  const EigenSupport::SpMatD &surfaceHessian,
  EigenSupport::SpMatD &simulationHessian) const
{
  Profiling::ScopedProfileSection pullbackProfile(SurfaceIPCProfileSections::kAdapterPullbackHessian);
  simulationHessian = surfaceFromSimulationDispMap_.transpose() * surfaceHessian * surfaceFromSimulationDispMap_;
}

double MappedSurfacePotentialEnergy::func(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterFunc);
  const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);
  return computeSurfaceEnergy(surfacePositions);
}

void MappedSurfacePotentialEnergy::gradient(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterGradient);
  const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);

  VXd surfaceGradient = VXd::Zero(surfaceRestPositions_.size());
  computeSurfaceGradient(surfacePositions, surfaceGradient);

  simulationGradient = pullbackSurfaceGradient(surfaceGradient);
}

void MappedSurfacePotentialEnergy::computeSurfaceFuncGrad(
  EigenSupport::ConstRefVecXd surfacePositions,
  double &surfaceEnergy,
  EigenSupport::RefVecXd surfaceGradient) const
{
  surfaceEnergy = computeSurfaceEnergy(surfacePositions);
  computeSurfaceGradient(surfacePositions, surfaceGradient);
}

void MappedSurfacePotentialEnergy::computeSurfaceAll(
  EigenSupport::ConstRefVecXd surfacePositions,
  double &surfaceEnergy,
  EigenSupport::RefVecXd surfaceGradient,
  EigenSupport::SpMatD &surfaceHessian) const
{
  surfaceEnergy = computeSurfaceEnergy(surfacePositions);
  computeSurfaceGradient(surfacePositions, surfaceGradient);
  computeSurfaceHessian(surfacePositions, surfaceHessian);
}

void MappedSurfacePotentialEnergy::computeSurfaceGradHessian(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::RefVecXd surfaceGradient,
  EigenSupport::SpMatD &surfaceHessian) const
{
  computeSurfaceGradient(surfacePositions, surfaceGradient);
  computeSurfaceHessian(surfacePositions, surfaceHessian);
}

double MappedSurfacePotentialEnergy::func_grad(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterGradient);
  const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);

  double surfaceEnergy = 0.0;
  VXd surfaceGradient = VXd::Zero(surfaceRestPositions_.size());
  computeSurfaceFuncGrad(surfacePositions, surfaceEnergy, surfaceGradient);

  simulationGradient = pullbackSurfaceGradient(surfaceGradient);
  return surfaceEnergy;
}

double MappedSurfacePotentialEnergy::func_grad_hessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient,
  EigenSupport::SpMatD &simulationHessian) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterHessianDirect);
  const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);

  double surfaceEnergy = 0.0;
  VXd surfaceGradient = VXd::Zero(surfaceRestPositions_.size());
  SpMatD surfaceHessian(surfaceRestPositions_.size(), surfaceRestPositions_.size());
  computeSurfaceAll(surfacePositions, surfaceEnergy, surfaceGradient, surfaceHessian);

  simulationGradient = pullbackSurfaceGradient(surfaceGradient);
  pullbackSurfaceHessian(surfaceHessian, simulationHessian);
  return surfaceEnergy;
}

void MappedSurfacePotentialEnergy::gradient_hessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient,
  EigenSupport::SpMatD &simulationHessian) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterHessianDirect);
  const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);

  VXd surfaceGradient = VXd::Zero(surfaceRestPositions_.size());
  SpMatD surfaceHessian(surfaceRestPositions_.size(), surfaceRestPositions_.size());
  computeSurfaceGradHessian(surfacePositions, surfaceGradient, surfaceHessian);

  simulationGradient = pullbackSurfaceGradient(surfaceGradient);
  pullbackSurfaceHessian(surfaceHessian, simulationHessian);
}

void MappedSurfacePotentialEnergy::hessianInPlace(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::SpMatD &simulationHessian) const
{
  hessian(simulationDisplacements, simulationHessian);
}

void MappedSurfacePotentialEnergy::hessianAlloc(EigenSupport::SpMatD &simulationHessian) const
{
  simulationHessian.resize(getNumDOFs(), getNumDOFs());
  simulationHessian.setZero();
}

void MappedSurfacePotentialEnergy::hessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::SpMatD &simulationHessian) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterHessianDirect);
  const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);

  SpMatD surfaceHessian(surfaceRestPositions_.size(), surfaceRestPositions_.size());
  computeSurfaceHessian(surfacePositions, surfaceHessian);

  pullbackSurfaceHessian(surfaceHessian, simulationHessian);
}

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
