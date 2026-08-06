/*
copyright to Bohan Wang
*/

#include "floor/floorContactEnergy.h"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace pgo
{
namespace Contact
{
namespace Floor
{

namespace
{
int floorAxisToIndex(FloorAxis axis)
{
  switch (axis) {
    case FloorAxis::X:
      return 0;
    case FloorAxis::Y:
      return 1;
    case FloorAxis::Z:
      return 2;
    default:
      throw std::invalid_argument("FloorPenaltyParameters.floorAxis must be X, Y, or Z.");
  }
}

// Sign that orients the penalty so that dzEff < 0 always means "on the
// forbidden side": KEEP_ABOVE penalizes pos < floorHeight, KEEP_BELOW penalizes
// pos > floorHeight.
double floorSideToSign(FloorSide side)
{
  switch (side) {
    case FloorSide::KEEP_ABOVE:
      return 1.0;
    case FloorSide::KEEP_BELOW:
      return -1.0;
    default:
      throw std::invalid_argument("FloorPenaltyParameters.floorSide must be KEEP_ABOVE or KEEP_BELOW.");
  }
}
}  // namespace

FloorContactEnergy::FloorContactEnergy(
  const EigenSupport::MXd &surfaceRestVertices,
  const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
  const FloorPenaltyParameters &params):
  dofMap_(surfaceRestVertices, surfaceFromSimulationDispMap),
  params_(params)
{
  (void)floorAxisToIndex(params_.floorAxis);
  (void)floorSideToSign(params_.floorSide);
  if (!std::isfinite(params_.floorHeight))
    throw std::invalid_argument("FloorPenaltyParameters.floorHeight must be finite.");
  if (!std::isfinite(params_.floorKappa))
    throw std::invalid_argument("FloorPenaltyParameters.floorKappa must be finite.");
}

double FloorContactEnergy::func(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  return computeSurfaceEnergy(surfacePositions);
}

void FloorContactEnergy::gradient(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient) const
{
  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);

  EigenSupport::VXd surfaceGradient = EigenSupport::VXd::Zero(dofMap_.numSurfaceDofs());
  computeSurfaceGradient(surfacePositions, surfaceGradient);

  simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
}

void FloorContactEnergy::hessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::SpMatD &simulationHessian) const
{
  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);

  EigenSupport::SpMatD surfaceHessian(dofMap_.numSurfaceDofs(), dofMap_.numSurfaceDofs());
  computeSurfaceHessian(surfacePositions, surfaceHessian);

  dofMap_.pullbackHessian(surfaceHessian, simulationHessian);
}

void FloorContactEnergy::hessianInPlace(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::SpMatD &simulationHessian) const
{
  hessian(simulationDisplacements, simulationHessian);
}

void FloorContactEnergy::hessianAlloc(EigenSupport::SpMatD &simulationHessian) const
{
  simulationHessian.resize(dofMap_.numSimulationDofs(), dofMap_.numSimulationDofs());
  simulationHessian.setZero();
}

double FloorContactEnergy::funcGradient(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient) const
{
  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);

  const double surfaceEnergy = computeSurfaceEnergy(surfacePositions);
  EigenSupport::VXd surfaceGradient = EigenSupport::VXd::Zero(dofMap_.numSurfaceDofs());
  computeSurfaceGradient(surfacePositions, surfaceGradient);

  simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
  return surfaceEnergy;
}

double FloorContactEnergy::funcGradientHessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient,
  EigenSupport::SpMatD &simulationHessian) const
{
  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);

  double surfaceEnergy = 0.0;
  EigenSupport::VXd surfaceGradient = EigenSupport::VXd::Zero(dofMap_.numSurfaceDofs());
  EigenSupport::SpMatD surfaceHessian(dofMap_.numSurfaceDofs(), dofMap_.numSurfaceDofs());
  computeSurfaceAll(surfacePositions, surfaceEnergy, surfaceGradient, surfaceHessian);

  simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
  dofMap_.pullbackHessian(surfaceHessian, simulationHessian);
  return surfaceEnergy;
}

void FloorContactEnergy::gradientHessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient,
  EigenSupport::SpMatD &simulationHessian) const
{
  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);

  EigenSupport::VXd surfaceGradient = EigenSupport::VXd::Zero(dofMap_.numSurfaceDofs());
  EigenSupport::SpMatD surfaceHessian(dofMap_.numSurfaceDofs(), dofMap_.numSurfaceDofs());
  computeSurfaceGradient(surfacePositions, surfaceGradient);
  computeSurfaceHessian(surfacePositions, surfaceHessian);

  simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
  dofMap_.pullbackHessian(surfaceHessian, simulationHessian);
}

void FloorContactEnergy::getDOFs(std::vector<int> &dofs) const
{
  dofs = dofMap_.simulationDofs();
}

int FloorContactEnergy::getNumDOFs() const
{
  return dofMap_.numSimulationDofs();
}

void FloorContactEnergy::setFloorHeight(double h)
{
  if (!std::isfinite(h))
    throw std::invalid_argument("FloorPenaltyParameters.floorHeight must be finite.");
  params_.floorHeight = h;
}

double FloorContactEnergy::floorHeight() const
{
  return params_.floorHeight;
}

double FloorContactEnergy::computeSurfaceEnergy(EigenSupport::ConstRefVecXd surfacePositions) const
{
  const int axis = floorAxisToIndex(params_.floorAxis);
  const double sideSign = floorSideToSign(params_.floorSide);
  double energy = 0.0;
  for (int vi = 0; vi < surfacePositions.size() / 3; ++vi) {
    const double dzEff = sideSign * (surfacePositions[3 * vi + axis] - params_.floorHeight);
    if (dzEff < 0.0)
      energy += 0.5 * params_.floorKappa * dzEff * dzEff;
  }
  return energy;
}

void FloorContactEnergy::computeSurfaceGradient(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::RefVecXd surfaceGradient) const
{
  const int axis = floorAxisToIndex(params_.floorAxis);
  const double sideSign = floorSideToSign(params_.floorSide);
  surfaceGradient.setZero();
  for (int vi = 0; vi < surfacePositions.size() / 3; ++vi) {
    const double dzEff = sideSign * (surfacePositions[3 * vi + axis] - params_.floorHeight);
    if (dzEff < 0.0)
      surfaceGradient[3 * vi + axis] = params_.floorKappa * dzEff * sideSign;
  }
}

void FloorContactEnergy::computeSurfaceHessian(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::SpMatD &surfaceHessian) const
{
  const int axis = floorAxisToIndex(params_.floorAxis);
  const double sideSign = floorSideToSign(params_.floorSide);
  std::vector<EigenSupport::TripletD> triplets;
  triplets.reserve(static_cast<std::size_t>(surfacePositions.size() / 3));
  for (int vi = 0; vi < surfacePositions.size() / 3; ++vi) {
    const double dzEff = sideSign * (surfacePositions[3 * vi + axis] - params_.floorHeight);
    if (dzEff < 0.0) {
      const int row = 3 * vi + axis;
      triplets.emplace_back(row, row, params_.floorKappa);
    }
  }

  surfaceHessian.resize(surfacePositions.size(), surfacePositions.size());
  surfaceHessian.setFromTriplets(triplets.begin(), triplets.end());
}

void FloorContactEnergy::computeSurfaceAll(
  EigenSupport::ConstRefVecXd surfacePositions,
  double &surfaceEnergy,
  EigenSupport::RefVecXd surfaceGradient,
  EigenSupport::SpMatD &surfaceHessian) const
{
  surfaceEnergy = computeSurfaceEnergy(surfacePositions);
  computeSurfaceGradient(surfacePositions, surfaceGradient);
  computeSurfaceHessian(surfacePositions, surfaceHessian);
}

}  // namespace Floor
}  // namespace Contact
}  // namespace pgo
