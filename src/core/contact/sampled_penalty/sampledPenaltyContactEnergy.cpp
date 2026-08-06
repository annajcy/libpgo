/*
  Long-lived sampled penalty contact energy.
*/

#include "sampled_penalty/sampledPenaltyContactEnergy.h"

#include "sampled_penalty/sampledPenaltyEvaluationBundle.h"
#include "sampled_penalty/kernels/pointPenetrationEnergy.h"
#include "sampled_penalty/kernels/pointTrianglePairCouplingEnergyWithCollision.h"

#include <stdexcept>
#include <utility>

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{
namespace
{
Mesh::TriMeshGeo makeSurfaceMesh(
  const EigenSupport::MXd &surfaceRestVertices,
  const EigenSupport::MXi &surfaceTriangles)
{
  if (surfaceRestVertices.cols() != 3)
    throw std::invalid_argument("SampledPenaltyContactEnergy rest vertices must have shape (#vertices, 3).");
  if (surfaceTriangles.cols() != 3)
    throw std::invalid_argument("SampledPenaltyContactEnergy surface triangles must have shape (#triangles, 3).");
  return Mesh::matricesToTriMeshGeo(surfaceRestVertices, surfaceTriangles);
}
}  // namespace

SampledPenaltyContactEnergy::SampledPenaltyContactEnergy(
  const EigenSupport::MXd &surfaceRestVertices,
  const EigenSupport::MXi &surfaceTriangles,
  const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
  const SampledPenaltyContactEnergyOptions &options,
  std::vector<Mesh::TriMeshGeo> externalSurfaces):
  dofMap_(surfaceRestVertices, surfaceFromSimulationDispMap),
  options_(options),
  builder_(
    makeSurfaceMesh(surfaceRestVertices, surfaceTriangles),
    dofMap_.numSurfaceDofs(),
    options_.params,
    std::move(externalSurfaces),
    {},
    {})
{
  if (options_.friction)
    frictionState_.emplace(*options_.friction);
}

SampledPenaltyContactEnergy::~SampledPenaltyContactEnergy() = default;

void SampledPenaltyContactEnergy::beginStep(const NonlinearOptimization::StepState &state)
{
  if (frictionState_) {
    if (state.previousX == nullptr)
      throw std::invalid_argument("SampledPenaltyContactEnergy::beginStep requires previousX when friction is enabled.");
    if (state.timestep <= 0.0)
      throw std::invalid_argument("SampledPenaltyContactEnergy::beginStep requires a positive timestep when friction is enabled.");

    EigenSupport::VXd previousSurfacePositions = dofMap_.surfacePositions(*state.previousX);
    NonlinearOptimization::StepState mapped = state;
    mapped.previousX = &previousSurfacePositions;
    frictionState_->beginStep(mapped, dofMap_.numSurfaceDofs());
  }

  const EigenSupport::VXd *bundleX = state.currentX != nullptr ? state.currentX : state.previousX;
  if (bundleX != nullptr)
    stepBundle_ = buildBundle(dofMap_.surfacePositions(*bundleX));
  else
    stepBundle_.reset();
}

void SampledPenaltyContactEnergy::updateExternalSurface(int index, const Mesh::TriMeshGeo &surface)
{
  builder_.updateExternalSurface(index, surface);
  stepBundle_.reset();
}

SampledPenaltyEnergyConfigurator SampledPenaltyContactEnergy::makeConfigurator() const
{
  SampledPenaltyEnergyConfigurator configurator;
  configurator.configureExternal = [this](PointPenetrationEnergy &energy) {
    configureExternalEnergy(energy);
  };
  configurator.configureSelf = [this](
                                PointTrianglePairCouplingEnergyWithCollision &energy,
                                EigenSupport::ConstRefVecXd state) {
    configureSelfEnergy(energy, state);
  };
  return configurator;
}

std::unique_ptr<SampledPenaltyEvaluationBundle> SampledPenaltyContactEnergy::buildBundle(
  EigenSupport::ConstRefVecXd surfacePositions) const
{
  return builder_.buildFromPositions(surfacePositions, makeConfigurator());
}

const SampledPenaltyEvaluationBundle &SampledPenaltyContactEnergy::evaluationBundle(
  EigenSupport::ConstRefVecXd surfacePositions,
  std::unique_ptr<SampledPenaltyEvaluationBundle> &fallbackBundle) const
{
  if (stepBundle_)
    return *stepBundle_;

  fallbackBundle = buildBundle(surfacePositions);
  return *fallbackBundle;
}

void SampledPenaltyContactEnergy::configureExternalEnergy(PointPenetrationEnergy &energy) const
{
  energy.setComputePosFunction([](const EigenSupport::V3d &x, EigenSupport::V3d &p, int) {
    p = x;
  });
  energy.setComputeLastPosFunction([](const EigenSupport::V3d &x, EigenSupport::V3d &p, int) {
    p = x;
  });
  energy.setCoeff(options_.params.stiffness);
  energy.setFrictionCoeff(0.0);
  energy.setTimestep(0.0);
  energy.setVelEps(0.0);
  if (frictionState_)
    frictionState_->configureExternalSurfacePositions(energy);
}

void SampledPenaltyContactEnergy::configureSelfEnergy(
  PointTrianglePairCouplingEnergyWithCollision &energy,
  EigenSupport::ConstRefVecXd surfacePositions) const
{
  energy.setToPosFunction([](const EigenSupport::V3d &x, EigenSupport::V3d &p, int) {
    p = x;
  });
  energy.setToLastPosFunction([](const EigenSupport::V3d &x, EigenSupport::V3d &p, int) {
    p = x;
  });
  energy.setCoeff(options_.params.stiffness);
  energy.setFrictionCoeff(0.0);
  energy.setTimestep(0.0);
  energy.setVelEps(0.0);
  if (frictionState_)
    frictionState_->configureSelfSurfacePositions(energy);
  energy.computeClosestPosition(surfacePositions.data());
}

double SampledPenaltyContactEnergy::func(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  std::unique_ptr<SampledPenaltyEvaluationBundle> fallbackBundle;
  const SampledPenaltyEvaluationBundle &bundle = evaluationBundle(surfacePositions, fallbackBundle);
  return evaluator_.func(bundle, surfacePositions);
}

void SampledPenaltyContactEnergy::gradient(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient) const
{
  if (simulationGradient.size() != dofMap_.numSimulationDofs())
    throw std::invalid_argument("SampledPenaltyContactEnergy gradient vector has unexpected size.");

  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  std::unique_ptr<SampledPenaltyEvaluationBundle> fallbackBundle;
  const SampledPenaltyEvaluationBundle &bundle = evaluationBundle(surfacePositions, fallbackBundle);
  EigenSupport::VXd surfaceGradient = EigenSupport::VXd::Zero(dofMap_.numSurfaceDofs());
  evaluator_.gradient(bundle, surfacePositions, surfaceGradient);
  simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
}

void SampledPenaltyContactEnergy::hessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::SpMatD &simulationHessian) const
{
  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  std::unique_ptr<SampledPenaltyEvaluationBundle> fallbackBundle;
  const SampledPenaltyEvaluationBundle &bundle = evaluationBundle(surfacePositions, fallbackBundle);
  EigenSupport::SpMatD surfaceHessian;
  evaluator_.hessian(bundle, surfacePositions, surfaceHessian);
  dofMap_.pullbackHessian(surfaceHessian, simulationHessian);
}

void SampledPenaltyContactEnergy::hessianInPlace(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::SpMatD &simulationHessian) const
{
  hessian(simulationDisplacements, simulationHessian);
}

void SampledPenaltyContactEnergy::hessianAlloc(EigenSupport::SpMatD &simulationHessian) const
{
  simulationHessian.resize(getNumDOFs(), getNumDOFs());
  simulationHessian.setZero();
}

double SampledPenaltyContactEnergy::funcGradient(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient) const
{
  if (simulationGradient.size() != dofMap_.numSimulationDofs())
    throw std::invalid_argument("SampledPenaltyContactEnergy gradient vector has unexpected size.");

  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  std::unique_ptr<SampledPenaltyEvaluationBundle> fallbackBundle;
  const SampledPenaltyEvaluationBundle &bundle = evaluationBundle(surfacePositions, fallbackBundle);
  EigenSupport::VXd surfaceGradient = EigenSupport::VXd::Zero(dofMap_.numSurfaceDofs());
  const double value = evaluator_.funcGradient(bundle, surfacePositions, surfaceGradient);
  simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
  return value;
}

double SampledPenaltyContactEnergy::funcGradientHessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient,
  EigenSupport::SpMatD &simulationHessian) const
{
  if (simulationGradient.size() != dofMap_.numSimulationDofs())
    throw std::invalid_argument("SampledPenaltyContactEnergy gradient vector has unexpected size.");

  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  std::unique_ptr<SampledPenaltyEvaluationBundle> fallbackBundle;
  const SampledPenaltyEvaluationBundle &bundle = evaluationBundle(surfacePositions, fallbackBundle);
  EigenSupport::VXd surfaceGradient = EigenSupport::VXd::Zero(dofMap_.numSurfaceDofs());
  EigenSupport::SpMatD surfaceHessian;
  const double value = evaluator_.funcGradientHessian(bundle, surfacePositions, surfaceGradient, surfaceHessian);
  simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
  dofMap_.pullbackHessian(surfaceHessian, simulationHessian);
  return value;
}

void SampledPenaltyContactEnergy::gradientHessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient,
  EigenSupport::SpMatD &simulationHessian) const
{
  if (simulationGradient.size() != dofMap_.numSimulationDofs())
    throw std::invalid_argument("SampledPenaltyContactEnergy gradient vector has unexpected size.");

  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  std::unique_ptr<SampledPenaltyEvaluationBundle> fallbackBundle;
  const SampledPenaltyEvaluationBundle &bundle = evaluationBundle(surfacePositions, fallbackBundle);
  EigenSupport::VXd surfaceGradient = EigenSupport::VXd::Zero(dofMap_.numSurfaceDofs());
  EigenSupport::SpMatD surfaceHessian;
  evaluator_.gradientHessian(bundle, surfacePositions, surfaceGradient, surfaceHessian);
  simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
  dofMap_.pullbackHessian(surfaceHessian, simulationHessian);
}

void SampledPenaltyContactEnergy::getDOFs(std::vector<int> &dofs) const
{
  dofs = dofMap_.simulationDofs();
}

int SampledPenaltyContactEnergy::getNumDOFs() const
{
  return dofMap_.numSimulationDofs();
}

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
