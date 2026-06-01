/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "deformationModelEnergy.h"

#include "deformationModelAssembler.h"
#include "scopedProfileSection.h"
#include "simulationMesh.h"
#include "pgoLogging.h"

#include <numeric>

using namespace pgo;
using namespace pgo::NonlinearOptimization;
using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

namespace
{

ES::VXd assembleAbsolutePositions(ES::ConstRefVecXd x, const ES::VXd &restPosition, int offset, int)
{
  return restPosition + x.segment(offset, restPosition.size());
}

ES::VXd assembleDirectionSlice(ES::ConstRefVecXd dx, int offset, int numDOFs)
{
  return ES::VXd(Eigen::Map<const ES::VXd>(dx.data() + offset, numDOFs));
}

const char *meshTypeName(pgo::SolidDeformationModel::SimulationMeshType meshType)
{
  using pgo::SolidDeformationModel::SimulationMeshType;
  switch (meshType) {
  case SimulationMeshType::TET:
    return "TET";
  case SimulationMeshType::CUBIC:
    return "CUBIC";
  case SimulationMeshType::TRIANGLE:
    return "TRIANGLE";
  case SimulationMeshType::EDGE_QUAD:
    return "EDGE_QUAD";
  case SimulationMeshType::SHELL:
    return "SHELL";
  default:
    return "UNKNOWN";
  }
}

}  // namespace

DeformationModelEnergy::DeformationModelEnergy(std::unique_ptr<DeformationModelAssembler> fma,
  int offset, bool enableMaterialMaxStep):
  forceModelAssembler(std::move(fma)),
  restPosition(std::make_unique<ES::VXd>(
    forceModelAssembler->getDeformationModelManager().buildRestPosition())),
  enableMaterialMaxStep_(enableMaterialMaxStep)
{
  allDOFs.resize(forceModelAssembler->getNumDOFs());
  std::iota(allDOFs.begin(), allDOFs.end(), offset);
}

DeformationModelEnergy::~DeformationModelEnergy()
{
}

double DeformationModelEnergy::func(EigenSupport::ConstRefVecXd x) const
{
  Profiling::ScopedProfileSection scopedProfile("material.energy");
  ES::VXd p = *restPosition + x.segment(allDOFs[0], restPosition->size());
  return forceModelAssembler->computeEnergy(p.data());
}

void DeformationModelEnergy::gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const
{
  Profiling::ScopedProfileSection scopedProfile("material.gradient");
  ES::VXd p = *restPosition + x.segment(allDOFs[0], restPosition->size());
  forceModelAssembler->computeGradient(p.data(), grad.data());
}

void DeformationModelEnergy::hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  Profiling::ScopedProfileSection scopedProfile("material.hessian");
  ES::VXd p = *restPosition + x.segment(allDOFs[0], restPosition->size());
  forceModelAssembler->computeHessian(p.data(), hess);
}

void DeformationModelEnergy::hessianAlloc(EigenSupport::SpMatD &hess) const
{
  hess = forceModelAssembler->getHessianTemplate();
}

NonlinearOptimization::MaxStepResult DeformationModelEnergy::computeMaxStepLimit(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx) const
{
  Profiling::ScopedProfileSection scopedProfile("material.max_step");
  if (!enableMaterialMaxStep_) {
    return NonlinearOptimization::MaxStepResult::unconstrained();
  }

  const int offset = allDOFs.empty() ? 0 : allDOFs[0];
  const int numDOFs = getNumDOFs();

  const ES::VXd dxLocal = assembleDirectionSlice(dx, offset, numDOFs);
  if (dxLocal.size() == 0 || dxLocal.squaredNorm() == 0.0) {
    return NonlinearOptimization::MaxStepResult::unconstrained();
  }

  const ES::VXd absolutePositions = assembleAbsolutePositions(x, *restPosition, offset, numDOFs);
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

  return NonlinearOptimization::MaxStepResult::material(maxStepSize);
}
