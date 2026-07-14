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

using namespace pgo;
using namespace pgo::NonlinearOptimization;
using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

namespace
{

void fillAbsolutePositions(ES::ConstRefVecXd x, const ES::VXd &restPosition, int offset, ES::VXd &out)
{
  out.noalias() = restPosition + x.segment(offset, restPosition.size());
}

void fillDirectionSlice(ES::ConstRefVecXd dx, int offset, int numDOFs, ES::VXd &out)
{
  out = Eigen::Map<const ES::VXd>(dx.data() + offset, numDOFs);
}

}  // namespace

DeformationModelEnergy::DeformationModelEnergy(std::unique_ptr<DeformationModelAssembler> fma,
  int offset, bool enableMaterialMaxStep):
  forceModelAssembler(std::move(fma)),
  restPosition(std::make_unique<ES::VXd>(forceModelAssembler->getRestPosition())),
  absolutePositionScratch_([this]() { return ES::VXd(forceModelAssembler->getNumDOFs()); }),
  directionScratch_([this]() { return ES::VXd(forceModelAssembler->getNumDOFs()); }),
  enableMaterialMaxStep_(enableMaterialMaxStep)
{
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
  Profiling::ScopedProfileSection scopedProfile("material.energy");
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(x, *restPosition, allDOFs[0], p);
  return forceModelAssembler->computeEnergy(p.data());
}

void DeformationModelEnergy::gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const
{
  Profiling::ScopedProfileSection scopedProfile("material.gradient");
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(x, *restPosition, allDOFs[0], p);
  forceModelAssembler->computeGradient(p.data(), grad.data());
}

void DeformationModelEnergy::hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  Profiling::ScopedProfileSection scopedProfile("material.hessian");
  ES::VXd &p = absolutePositionScratch();
  fillAbsolutePositions(x, *restPosition, allDOFs[0], p);
  forceModelAssembler->computeHessian(p.data(), hess);
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

  const int offset = allDOFs.empty() ? 0 : allDOFs[0];
  const int numDOFs = getNumDOFs();

  ES::VXd &dxLocal = directionScratch();
  fillDirectionSlice(dx, offset, numDOFs, dxLocal);
  if (dxLocal.size() == 0 || dxLocal.squaredNorm() == 0.0) {
    return NonlinearOptimization::StepConstraint{};
  }

  ES::VXd &absolutePositions = absolutePositionScratch();
  fillAbsolutePositions(x, *restPosition, offset, absolutePositions);
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
