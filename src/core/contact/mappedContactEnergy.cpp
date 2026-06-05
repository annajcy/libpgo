/*
  Generic mapped contact adapter.
*/

#include "mappedContactEnergy.h"

#include "evaluationStateAwareEnergy.h"
#include "lineSearchAwareEnergy.h"
#include "stepAwareEnergy.h"
#include "stepDependentEnergy.h"

#include <numeric>
#include <stdexcept>

namespace pgo
{
namespace Contact
{
namespace ES = pgo::EigenSupport;
namespace NO = pgo::NonlinearOptimization;

namespace
{
ES::VXd flattenSurfaceRestVertices(const ES::MXd &surfaceRestVertices)
{
  if (surfaceRestVertices.cols() != 3)
    throw std::invalid_argument("MappedContactEnergy surfaceRestVertices must have shape (#vertices, 3).");
  if (surfaceRestVertices.rows() <= 0)
    throw std::invalid_argument("MappedContactEnergy surfaceRestVertices must contain at least one vertex.");

  ES::VXd rest(surfaceRestVertices.rows() * 3);
  for (Eigen::Index vi = 0; vi < surfaceRestVertices.rows(); vi++)
    rest.segment<3>(vi * 3) = surfaceRestVertices.row(vi).transpose();
  return rest;
}

const NO::EvaluationStateAwareEnergy *asConstEvaluationAware(const std::shared_ptr<StatefulContactEnergy> &energy)
{
  return dynamic_cast<const NO::EvaluationStateAwareEnergy *>(energy.get());
}

const NO::LineSearchAwareEnergy *asConstLineSearchAware(const std::shared_ptr<StatefulContactEnergy> &energy)
{
  return dynamic_cast<const NO::LineSearchAwareEnergy *>(energy.get());
}

NO::StepAwareEnergy *asStepAware(const std::shared_ptr<StatefulContactEnergy> &energy)
{
  return dynamic_cast<NO::StepAwareEnergy *>(energy.get());
}
}  // namespace

MappedContactEnergy::MappedContactEnergy(
  const ES::MXd &surfaceRestVertices,
  const ES::SpMatD &surfaceFromSimulationDispMap,
  std::shared_ptr<StatefulContactEnergy> surfacePositionEnergy):
  surfacePositionEnergy_(std::move(surfacePositionEnergy)),
  surfaceRestPositions_(flattenSurfaceRestVertices(surfaceRestVertices)),
  surfaceFromSimulationDispMap_(surfaceFromSimulationDispMap)
{
  if (!surfacePositionEnergy_)
    throw std::invalid_argument("MappedContactEnergy requires a surface-position contact energy.");
  if (surfaceFromSimulationDispMap_.rows() != surfaceRestPositions_.size())
    throw std::invalid_argument("MappedContactEnergy W rows must equal 3 * #surface vertices.");
  if (surfaceFromSimulationDispMap_.cols() <= 0)
    throw std::invalid_argument("MappedContactEnergy W must contain at least one simulation DOF column.");
  if (surfacePositionEnergy_->getNumDOFs() != surfaceRestPositions_.size())
    throw std::invalid_argument("MappedContactEnergy child energy DOF count must equal 3 * #surface vertices.");

  simulationDofs_.resize(static_cast<std::size_t>(surfaceFromSimulationDispMap_.cols()));
  std::iota(simulationDofs_.begin(), simulationDofs_.end(), 0);
}

ContactModelKind MappedContactEnergy::contactModelKind() const
{
  return surfacePositionEnergy_->contactModelKind();
}

void MappedContactEnergy::validateSimulationDisplacementSize(ES::ConstRefVecXd simulationDisplacements) const
{
  if (simulationDisplacements.size() != surfaceFromSimulationDispMap_.cols())
    throw std::invalid_argument("MappedContactEnergy simulation displacement size does not match W column count.");
}

ES::VXd MappedContactEnergy::surfaceDisplacements(ES::ConstRefVecXd simulationDisplacements) const
{
  validateSimulationDisplacementSize(simulationDisplacements);
  return ES::VXd(surfaceFromSimulationDispMap_ * simulationDisplacements);
}

ES::VXd MappedContactEnergy::surfacePositions(ES::ConstRefVecXd simulationDisplacements) const
{
  return surfaceRestPositions_ + surfaceDisplacements(simulationDisplacements);
}

ES::VXd MappedContactEnergy::pullbackGradient(ES::ConstRefVecXd surfaceGradient) const
{
  return surfaceFromSimulationDispMap_.transpose() * surfaceGradient;
}

void MappedContactEnergy::pullbackHessian(const ES::SpMatD &surfaceHessian, ES::SpMatD &simulationHessian) const
{
  simulationHessian = surfaceFromSimulationDispMap_.transpose() * surfaceHessian * surfaceFromSimulationDispMap_;
}

double MappedContactEnergy::func(ES::ConstRefVecXd simulationDisplacements) const
{
  return surfacePositionEnergy_->func(surfacePositions(simulationDisplacements));
}

void MappedContactEnergy::gradient(ES::ConstRefVecXd simulationDisplacements, ES::RefVecXd simulationGradient) const
{
  const ES::VXd xSurface = surfacePositions(simulationDisplacements);
  ES::VXd surfaceGradient = ES::VXd::Zero(surfaceRestPositions_.size());
  surfacePositionEnergy_->gradient(xSurface, surfaceGradient);
  simulationGradient = pullbackGradient(surfaceGradient);
}

void MappedContactEnergy::hessianInPlace(ES::ConstRefVecXd simulationDisplacements, ES::SpMatD &simulationHessian) const
{
  hessian(simulationDisplacements, simulationHessian);
}

void MappedContactEnergy::hessianAlloc(ES::SpMatD &simulationHessian) const
{
  simulationHessian.resize(getNumDOFs(), getNumDOFs());
  simulationHessian.setZero();
}

void MappedContactEnergy::hessian(ES::ConstRefVecXd simulationDisplacements, ES::SpMatD &simulationHessian) const
{
  const ES::VXd xSurface = surfacePositions(simulationDisplacements);
  ES::SpMatD surfaceHessian;
  surfacePositionEnergy_->hessian(xSurface, surfaceHessian);
  pullbackHessian(surfaceHessian, simulationHessian);
}

double MappedContactEnergy::func_grad(ES::ConstRefVecXd simulationDisplacements, ES::RefVecXd simulationGradient) const
{
  const ES::VXd xSurface = surfacePositions(simulationDisplacements);
  ES::VXd surfaceGradient = ES::VXd::Zero(surfaceRestPositions_.size());
  const double energy = surfacePositionEnergy_->func_grad(xSurface, surfaceGradient);
  simulationGradient = pullbackGradient(surfaceGradient);
  return energy;
}

double MappedContactEnergy::func_grad_hessian(
  ES::ConstRefVecXd simulationDisplacements,
  ES::RefVecXd simulationGradient,
  ES::SpMatD &simulationHessian) const
{
  const ES::VXd xSurface = surfacePositions(simulationDisplacements);
  ES::VXd surfaceGradient = ES::VXd::Zero(surfaceRestPositions_.size());
  ES::SpMatD surfaceHessian;
  const double energy = surfacePositionEnergy_->func_grad_hessian(xSurface, surfaceGradient, surfaceHessian);
  simulationGradient = pullbackGradient(surfaceGradient);
  pullbackHessian(surfaceHessian, simulationHessian);
  return energy;
}

void MappedContactEnergy::gradient_hessian(
  ES::ConstRefVecXd simulationDisplacements,
  ES::RefVecXd simulationGradient,
  ES::SpMatD &simulationHessian) const
{
  const ES::VXd xSurface = surfacePositions(simulationDisplacements);
  ES::VXd surfaceGradient = ES::VXd::Zero(surfaceRestPositions_.size());
  ES::SpMatD surfaceHessian;
  surfacePositionEnergy_->gradient_hessian(xSurface, surfaceGradient, surfaceHessian);
  simulationGradient = pullbackGradient(surfaceGradient);
  pullbackHessian(surfaceHessian, simulationHessian);
}

void MappedContactEnergy::getDOFs(std::vector<int> &dofs) const
{
  dofs = simulationDofs_;
}

int MappedContactEnergy::getNumDOFs() const
{
  return static_cast<int>(surfaceFromSimulationDispMap_.cols());
}

int MappedContactEnergy::isHessianTopologyFixed() const
{
  return surfacePositionEnergy_->isHessianTopologyFixed();
}

void MappedEvaluationContactEnergy::prepareEvaluationState(ES::ConstRefVecXd simulationDisplacements) const
{
  auto *aware = asConstEvaluationAware(surfacePositionEnergy_);
  if (!aware)
    return;
  aware->prepareEvaluationState(surfacePositions(simulationDisplacements));
}

void MappedEvaluationContactEnergy::beginLineSearch(
  ES::ConstRefVecXd simulationDisplacements,
  ES::ConstRefVecXd simulationStep) const
{
  auto *aware = asConstLineSearchAware(surfacePositionEnergy_);
  if (!aware)
    return;
  validateSimulationDisplacementSize(simulationStep);
  aware->beginLineSearch(surfacePositions(simulationDisplacements), surfaceFromSimulationDispMap_ * simulationStep);
}

void MappedEvaluationContactEnergy::endLineSearch() const
{
  auto *aware = asConstLineSearchAware(surfacePositionEnergy_);
  if (aware)
    aware->endLineSearch();
}

double MappedEvaluationContactEnergy::maxValidLineSearchAlpha() const
{
  auto *aware = asConstLineSearchAware(surfacePositionEnergy_);
  return aware ? aware->maxValidLineSearchAlpha() : 1.0;
}

void MappedStepAwareContactEnergy::beginStep(const NO::StepState &state)
{
  auto *aware = asStepAware(surfacePositionEnergy_);
  if (!aware)
    return;

  ES::VXd previousSurface;
  NO::StepState mapped = state;
  if (state.previousX) {
    previousSurface = surfacePositions(*state.previousX);
    mapped.previousX = &previousSurface;
  }
  aware->beginStep(mapped);
}

std::shared_ptr<StatefulContactEnergy> makeMappedContactEnergy(
  const ES::MXd &surfaceRestVertices,
  const ES::SpMatD &surfaceFromSimulationDispMap,
  std::shared_ptr<StatefulContactEnergy> surfacePositionEnergy)
{
  if (dynamic_cast<NO::StepAwareEnergy *>(surfacePositionEnergy.get()) ||
    dynamic_cast<NO::StepDependentEnergy *>(surfacePositionEnergy.get())) {
    return std::make_shared<MappedStepAwareContactEnergy>(
      surfaceRestVertices, surfaceFromSimulationDispMap, std::move(surfacePositionEnergy));
  }
  if (dynamic_cast<NO::EvaluationStateAwareEnergy *>(surfacePositionEnergy.get()) ||
    dynamic_cast<NO::LineSearchAwareEnergy *>(surfacePositionEnergy.get())) {
    return std::make_shared<MappedEvaluationContactEnergy>(
      surfaceRestVertices, surfaceFromSimulationDispMap, std::move(surfacePositionEnergy));
  }
  return std::make_shared<MappedContactEnergy>(
    surfaceRestVertices, surfaceFromSimulationDispMap, std::move(surfacePositionEnergy));
}

}  // namespace Contact
}  // namespace pgo
