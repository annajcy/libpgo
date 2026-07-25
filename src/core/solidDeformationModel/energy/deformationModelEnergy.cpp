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

EigenSupport::VXd buildVertexRestPositions(const SimulationMesh &mesh)
{
  EigenSupport::VXd positions(mesh.getNumVertices() * 3);
  for (int vi = 0; vi < mesh.getNumVertices(); vi++) {
    double p[3];
    mesh.getVertex(vi, p);
    positions.segment<3>(vi * 3) = EigenSupport::V3d(p[0], p[1], p[2]);
  }
  return positions;
}

void fillAbsolutePositions(ES::ConstRefVecXd x, const ES::VXd &restDofs, ES::VXd &out)
{
  if (x.size() != restDofs.size()) {
    throw std::invalid_argument(
      "DeformationModelEnergy: local displacement size does not match the energy DOF count.");
  }
  out.noalias() = restDofs + x;
}

}  // namespace

DeformationModelEnergy::DeformationModelEnergy(
  std::unique_ptr<DeformationModelAssembler> fma,
  std::shared_ptr<MaterialParameters> materialParameters,
  int offset,
  bool enableMaterialMaxStep):
  forceModelAssembler(std::move(fma)),
  materialParameters_(std::move(materialParameters)),
  restDofs(std::make_unique<ES::VXd>(forceModelAssembler->getRestDofs())),
  vertexRestPositions(buildVertexRestPositions(
    *forceModelAssembler->getDeformationModelManager().getMesh())),
  absolutePositionScratch_([this]() { return ES::VXd(forceModelAssembler->getNumDOFs()); }),
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

double DeformationModelEnergy::func(EigenSupport::ConstRefVecXd x) const
{
  return func(x, materialParameters_->snapshot().view());
}

double DeformationModelEnergy::func(
  EigenSupport::ConstRefVecXd x, MaterialParameterEvaluationView state) const
{
  Profiling::ScopedProfileSection scopedProfile("material.energy");
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(x, *restDofs, p);
  return forceModelAssembler->computeEnergy(p.data(), state);
}

void DeformationModelEnergy::compute_dE_dp(
  ES::ConstRefVecXd displacement, ES::RefVecXd grad) const
{
  compute_dE_dp(displacement, materialParameters_->snapshot().view(), grad);
}

void DeformationModelEnergy::compute_dE_dp(
  ES::ConstRefVecXd displacement, MaterialParameterEvaluationView state, ES::RefVecXd grad) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restDofs, p);
  forceModelAssembler->compute_dE_dp(
    p.data(), state, grad.data());
}

void DeformationModelEnergy::compute_dE_de(
  ES::ConstRefVecXd displacement, ES::RefVecXd grad) const
{
  compute_dE_de(displacement, materialParameters_->snapshot().view(), grad);
}

void DeformationModelEnergy::compute_dE_de(
  ES::ConstRefVecXd displacement, MaterialParameterEvaluationView state, ES::RefVecXd grad) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restDofs, p);
  forceModelAssembler->compute_dE_de(
    p.data(), state, grad.data());
}

void DeformationModelEnergy::compute_d2E_dp2(
  ES::ConstRefVecXd displacement, ES::SpMatD &hess) const
{
  compute_d2E_dp2(displacement, materialParameters_->snapshot().view(), hess);
}

void DeformationModelEnergy::compute_d2E_dp2(
  ES::ConstRefVecXd displacement, MaterialParameterEvaluationView state, ES::SpMatD &hess) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restDofs, p);
  forceModelAssembler->compute_d2E_dp2(
    p.data(), state, hess);
}

void DeformationModelEnergy::compute_d2E_de2(
  ES::ConstRefVecXd displacement, ES::SpMatD &hess) const
{
  compute_d2E_de2(displacement, materialParameters_->snapshot().view(), hess);
}

void DeformationModelEnergy::compute_d2E_de2(
  ES::ConstRefVecXd displacement, MaterialParameterEvaluationView state, ES::SpMatD &hess) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restDofs, p);
  forceModelAssembler->compute_d2E_de2(
    p.data(), state, hess);
}

void DeformationModelEnergy::compute_d2E_dpde(
  ES::ConstRefVecXd displacement, ES::SpMatD &hess) const
{
  compute_d2E_dpde(
    displacement, materialParameters_->snapshot().view(), hess);
}

void DeformationModelEnergy::compute_d2E_dpde(
  ES::ConstRefVecXd displacement, MaterialParameterEvaluationView state, ES::SpMatD &hess) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restDofs, p);
  forceModelAssembler->compute_d2E_dpde(
    p.data(), state, hess);
}

void DeformationModelEnergy::compute_d2E_dudp(
  ES::ConstRefVecXd displacement, ES::SpMatD &mixedHessian) const
{
  compute_d2E_dudp(
    displacement, materialParameters_->snapshot().view(), mixedHessian);
}

void DeformationModelEnergy::compute_d2E_dudp(
  ES::ConstRefVecXd displacement, MaterialParameterEvaluationView state,
  ES::SpMatD &mixedHessian) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restDofs, p);
  forceModelAssembler->compute_d2E_dudp(
    p.data(), state, mixedHessian);
}

void DeformationModelEnergy::compute_d2E_dude(
  ES::ConstRefVecXd displacement, ES::SpMatD &mixedHessian) const
{
  compute_d2E_dude(
    displacement, materialParameters_->snapshot().view(), mixedHessian);
}

void DeformationModelEnergy::compute_d2E_dude(
  ES::ConstRefVecXd displacement, MaterialParameterEvaluationView state,
  ES::SpMatD &mixedHessian) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restDofs, p);
  forceModelAssembler->compute_d2E_dude(
    p.data(), state, mixedHessian);
}

void DeformationModelEnergy::computeVonMisesStresses(
  ES::ConstRefVecXd displacement, ES::RefVecXd elementStresses) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restDofs, p);
  forceModelAssembler->computeVonMisesStresses(
    p.data(), materialParameters_->snapshot().view(), elementStresses.data());
}

void DeformationModelEnergy::computeMaxStrains(
  ES::ConstRefVecXd displacement, ES::RefVecXd elementStrains) const
{
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(displacement, *restDofs, p);
  forceModelAssembler->computeMaxStrains(
    p.data(), materialParameters_->snapshot().view(), elementStrains.data());
}

void DeformationModelEnergy::gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const
{
  gradient(x, materialParameters_->snapshot().view(), grad);
}

void DeformationModelEnergy::gradient(
  EigenSupport::ConstRefVecXd x,
  MaterialParameterEvaluationView state,
  EigenSupport::RefVecXd grad) const
{
  Profiling::ScopedProfileSection scopedProfile("material.gradient");
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(x, *restDofs, p);
  forceModelAssembler->computeGradient(p.data(), state, grad.data());
}

void DeformationModelEnergy::hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  hessianInPlace(x, materialParameters_->snapshot().view(), hess);
}

void DeformationModelEnergy::hessianInPlace(
  EigenSupport::ConstRefVecXd x,
  MaterialParameterEvaluationView state,
  EigenSupport::SpMatD &hess) const
{
  Profiling::ScopedProfileSection scopedProfile("material.hessian");
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(x, *restDofs, p);
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

  if (dx.size() == 0 || dx.squaredNorm() == 0.0) {
    return NonlinearOptimization::StepConstraint{};
  }

  ES::VXd &absolutePositions = absolutePositionScratch();
  fillAbsolutePositions(x, *restDofs, absolutePositions);
  const auto observation = forceModelAssembler->computeMaxStepObservation(absolutePositions.data(), dx.data());
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
