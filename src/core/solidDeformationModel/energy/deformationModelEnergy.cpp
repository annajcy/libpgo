/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "energy/deformationModelEnergy.h"

#include "deformation/deformationModelAssembler.h"
#include "scopedProfileSection.h"
#include "simulation/simulationMesh.h"
#include "pgoLogging.h"

#include <algorithm>
#include <numeric>
#include <stdexcept>

using namespace pgo;
using namespace pgo::NonlinearOptimization;
using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

namespace
{

void fillAbsolutePositions(ES::ConstRefVecXd x, const ES::VXd &restPosition, ES::VXd &out)
{
  if (x.size() != restPosition.size()) {
    throw std::invalid_argument(
      "DeformationModelEnergy: local displacement size does not match the energy DOF count.");
  }
  out.noalias() = restPosition + x;
}

}  // namespace

DeformationModelEnergy::DeformationModelEnergy(
  std::unique_ptr<DeformationModelAssembler> fma,
  std::shared_ptr<MaterialParameters> materialParameters,
  int offset,
  bool enableMaterialMaxStep):
  forceModelAssembler(std::move(fma)),
  materialParameters_(std::move(materialParameters)),
  restPosition(std::make_unique<ES::VXd>(forceModelAssembler->getRestPosition())),
  absolutePositionScratch_([this]() { return ES::VXd(forceModelAssembler->getNumDOFs()); }),
  directionScratch_([this]() { return ES::VXd(forceModelAssembler->getNumDOFs()); }),
  enableMaterialMaxStep_(enableMaterialMaxStep)
{
  if (!materialParameters_)
    throw std::invalid_argument("DeformationModelEnergy requires material parameters.");
  if (materialParameters_->space().get() !=
    forceModelAssembler->materialParameterSpace().get()) {
    throw std::invalid_argument(
      "DeformationModelEnergy material parameters and assembler use different parameter spaces.");
  }
  allDOFs.resize(forceModelAssembler->getNumDOFs());
  std::iota(allDOFs.begin(), allDOFs.end(), offset);
}

DeformationModelEnergy::~DeformationModelEnergy()
{
}

ES::VXd &DeformationModelEnergy::absolutePositionScratch() const
{
  return absolutePositionScratch_.local();
}

ES::VXd &DeformationModelEnergy::directionScratch() const
{
  return directionScratch_.local();
}

double DeformationModelEnergy::func(EigenSupport::ConstRefVecXd x) const
{
  return func(x, materialParameters_->committedView());
}

double DeformationModelEnergy::func(
  EigenSupport::ConstRefVecXd x, MaterialStateView state) const
{
  Profiling::ScopedProfileSection scopedProfile("material.energy");
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(x, *restPosition, p);
  return forceModelAssembler->computeEnergy(p.data(), state);
}

void DeformationModelEnergy::computePlasticGradient(
  ES::ConstRefVecXd displacement, ES::RefVecXd grad) const
{
  computePlasticGradient(displacement, materialParameters_->committedView(), grad);
}

void DeformationModelEnergy::computePlasticGradient(
  ES::ConstRefVecXd displacement, MaterialStateView state, ES::RefVecXd grad) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restPosition, p);
  forceModelAssembler->computePlasticGradient(
    p.data(), state, grad.data());
}

void DeformationModelEnergy::computeElasticGradient(
  ES::ConstRefVecXd displacement, ES::RefVecXd grad) const
{
  computeElasticGradient(displacement, materialParameters_->committedView(), grad);
}

void DeformationModelEnergy::computeElasticGradient(
  ES::ConstRefVecXd displacement, MaterialStateView state, ES::RefVecXd grad) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restPosition, p);
  forceModelAssembler->computeElasticGradient(
    p.data(), state, grad.data());
}

void DeformationModelEnergy::computePlasticHessian(
  ES::ConstRefVecXd displacement, ES::SpMatD &hess) const
{
  computePlasticHessian(displacement, materialParameters_->committedView(), hess);
}

void DeformationModelEnergy::computePlasticHessian(
  ES::ConstRefVecXd displacement, MaterialStateView state, ES::SpMatD &hess) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restPosition, p);
  forceModelAssembler->computePlasticHessian(
    p.data(), state, hess);
}

void DeformationModelEnergy::computeElasticHessian(
  ES::ConstRefVecXd displacement, ES::SpMatD &hess) const
{
  computeElasticHessian(displacement, materialParameters_->committedView(), hess);
}

void DeformationModelEnergy::computeElasticHessian(
  ES::ConstRefVecXd displacement, MaterialStateView state, ES::SpMatD &hess) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restPosition, p);
  forceModelAssembler->computeElasticHessian(
    p.data(), state, hess);
}

void DeformationModelEnergy::computePlasticElasticHessian(
  ES::ConstRefVecXd displacement, ES::SpMatD &hess) const
{
  computePlasticElasticHessian(
    displacement, materialParameters_->committedView(), hess);
}

void DeformationModelEnergy::computePlasticElasticHessian(
  ES::ConstRefVecXd displacement, MaterialStateView state, ES::SpMatD &hess) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restPosition, p);
  forceModelAssembler->computePlasticElasticHessian(
    p.data(), state, hess);
}

void DeformationModelEnergy::computeDfDa(
  ES::ConstRefVecXd displacement, ES::SpMatD &jacobian) const
{
  computeDfDa(displacement, materialParameters_->committedView(), jacobian);
}

void DeformationModelEnergy::computeDfDa(
  ES::ConstRefVecXd displacement, MaterialStateView state, ES::SpMatD &jacobian) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restPosition, p);
  forceModelAssembler->compute_df_da(
    p.data(), state, jacobian);
}

void DeformationModelEnergy::computeDfDb(
  ES::ConstRefVecXd displacement, ES::SpMatD &jacobian) const
{
  computeDfDb(displacement, materialParameters_->committedView(), jacobian);
}

void DeformationModelEnergy::computeDfDb(
  ES::ConstRefVecXd displacement, MaterialStateView state, ES::SpMatD &jacobian) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restPosition, p);
  forceModelAssembler->compute_df_db(
    p.data(), state, jacobian);
}

void DeformationModelEnergy::computeVonMisesStresses(
  ES::ConstRefVecXd displacement, ES::RefVecXd elementStresses) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restPosition, p);
  forceModelAssembler->computeVonMisesStresses(
    p.data(), materialParameters_->committedView(), elementStresses.data());
}

void DeformationModelEnergy::computeMaxStrains(
  ES::ConstRefVecXd displacement, ES::RefVecXd elementStrains) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restPosition, p);
  forceModelAssembler->computeMaxStrains(
    p.data(), materialParameters_->committedView(), elementStrains.data());
}

void DeformationModelEnergy::gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const
{
  gradient(x, materialParameters_->committedView(), grad);
}

void DeformationModelEnergy::gradient(
  EigenSupport::ConstRefVecXd x,
  MaterialStateView state,
  EigenSupport::RefVecXd grad) const
{
  Profiling::ScopedProfileSection scopedProfile("material.gradient");
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(x, *restPosition, p);
  forceModelAssembler->computeGradient(p.data(), state, grad.data());
}

void DeformationModelEnergy::hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  hessianInPlace(x, materialParameters_->committedView(), hess);
}

void DeformationModelEnergy::hessianInPlace(
  EigenSupport::ConstRefVecXd x,
  MaterialStateView state,
  EigenSupport::SpMatD &hess) const
{
  Profiling::ScopedProfileSection scopedProfile("material.hessian");
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(x, *restPosition, p);
  forceModelAssembler->computeHessian(p.data(), state, hess);
}

void DeformationModelEnergy::hessianAlloc(EigenSupport::SpMatD &hess) const
{
  hess = forceModelAssembler->getHessianTemplate();
}

NonlinearOptimization::StepConstraint DeformationModelEnergy::computeMaxStepLimit(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx, StepConstraintSink *sink) const
{
  Profiling::ScopedProfileSection scopedProfile("material.max_step");
  if (!enableMaterialMaxStep_) {
    return NonlinearOptimization::StepConstraint{};
  }

  const int numDOFs = getNumDOFs();
  if (x.size() != numDOFs || dx.size() != numDOFs) {
    throw std::invalid_argument(
      "DeformationModelEnergy::computeMaxStepLimit: local state size does not match the energy DOF count.");
  }

  ES::VXd &dxLocal = directionScratch();
  dxLocal = dx;
  if (dxLocal.size() == 0 || dxLocal.squaredNorm() == 0.0) {
    return NonlinearOptimization::StepConstraint{};
  }

  ES::VXd &absolutePositions = absolutePositionScratch();
  fillAbsolutePositions(x, *restPosition, absolutePositions);
  const auto observation = forceModelAssembler->computeMaxStepObservation(absolutePositions.data(), dxLocal.data());
  const double maxStepSize = observation.alpha;

  if (maxStepSize < 1.0) {
    const auto meshType = forceModelAssembler->getDeformationModelManager().getMesh()->getElementType();

    if (!observation.hasIllegalInitialState && maxStepSize > 0.0 && maxStepSize < 0.01) {
      if (observation.limitingLocationId >= 0) {
        SPDLOG_LOGGER_WARN(Logging::lgr(),
          "material max step produced small materialFeasibleAlpha={} on meshType={} element={} location={}.",
          maxStepSize, meshTypeName(meshType), observation.limitingElementId, observation.limitingLocationId);
      }
      else {
        SPDLOG_LOGGER_WARN(Logging::lgr(),
          "material max step produced small materialFeasibleAlpha={} on meshType={} element={}.",
          maxStepSize, meshTypeName(meshType), observation.limitingElementId);
      }
    }

    if (auto logger = Logging::lgr(); logger && logger->should_log(spdlog::level::trace)) {
      if (observation.limitingLocationId >= 0) {
        SPDLOG_LOGGER_TRACE(logger,
          "material clamp: materialFeasibleAlpha={} meshType={} element={} location={}.",
          maxStepSize, meshTypeName(meshType), observation.limitingElementId, observation.limitingLocationId);
      }
      else {
        SPDLOG_LOGGER_TRACE(logger,
          "material clamp: materialFeasibleAlpha={} meshType={} element={}.",
          maxStepSize, meshTypeName(meshType), observation.limitingElementId);
      }
    }
  }

  NonlinearOptimization::StepConstraint c{NonlinearOptimization::StepSource::Material, maxStepSize};
  if (sink)
    sink->report(c);
  return c;
}
