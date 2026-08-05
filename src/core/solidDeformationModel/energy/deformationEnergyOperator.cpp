/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "energy/deformationEnergyOperator.h"

#include "deformation/deformationModelAssembler.h"
#include "formulations/formulation/formulation.h"
#include "material/runtime/materialBinding.h"
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
    positions.segment<3>(vi * 3) = mesh.getVertex(vi);
  }
  return positions;
}

void fillAbsolutePositions(ES::ConstRefVecXd x, const ES::VXd &restDofs, ES::VXd &out)
{
  if (x.size() != restDofs.size()) {
    throw std::invalid_argument(
      "DeformationEnergyOperator: local displacement size does not match the energy DOF count.");
  }
  out.noalias() = restDofs + x;
}

}  // namespace

DeformationEnergyOperator::DeformationEnergyOperator(
  const SimulationMesh &mesh,
  const MaterialBinding &materialBinding,
  const Formulation &formulation,
  const DeformationModelOptions &options):
  DeformationEnergyOperator(build(
    mesh, materialBinding, formulation, options))
{
}

DeformationEnergyOperator::BuildComponents DeformationEnergyOperator::build(
  const SimulationMesh &mesh,
  const MaterialBinding &materialBinding,
  const Formulation &formulation,
  const DeformationModelOptions &options)
{
  const int numElements = mesh.getNumElements();
  if (materialBinding.numElements() != numElements)
    throw std::invalid_argument(
      "DeformationEnergyOperator material binding element count does not match mesh.");

  ES::VXd elementWeights = options.elementWeights;
  if (elementWeights.size() == 0)
    elementWeights = ES::VXd::Ones(numElements);
  else if (elementWeights.size() != numElements)
    throw std::invalid_argument(
      "DeformationEnergyOperator element weight count does not match the mesh.");

  auto assembler = std::make_unique<DeformationModelAssembler>(
    mesh, materialBinding, formulation, options.projectHessianPSD,
    std::span<const double>(
      elementWeights.data(),
      static_cast<std::size_t>(elementWeights.size())));
  return BuildComponents{
    std::move(assembler), buildVertexRestPositions(mesh),
    options.enableMaterialMaxStep, options.dofOffset
  };
}

DeformationEnergyOperator::DeformationEnergyOperator(
  BuildComponents components):
  forceModelAssembler(std::move(components.assembler)),
  vertexRestPositions(std::move(components.vertexRestPositions)),
  absolutePositionScratch_(forceModelAssembler->getNumDOFs()),
  enableMaterialMaxStep_(components.enableMaterialMaxStep)
{
  allDOFs.resize(forceModelAssembler->getNumDOFs());
  std::iota(allDOFs.begin(), allDOFs.end(), components.dofOffset);
}

const EigenSupport::VXd &DeformationEnergyOperator::getRestDofs() const
{
  return forceModelAssembler->getRestDofs();
}

int DeformationEnergyOperator::getNumVertices() const
{
  return forceModelAssembler->getNumVertices();
}

int DeformationEnergyOperator::getNumElements() const
{
  return forceModelAssembler->getNumElements();
}

DeformationEnergyOperator::~DeformationEnergyOperator()
{
}

double DeformationEnergyOperator::func(
  EigenSupport::ConstRefVecXd x, MaterialStateView state) const
{
  Profiling::ScopedProfileSection scopedProfile("material.energy");
  ES::VXd &p = absolutePositionScratch_;
  fillAbsolutePositions(x, getRestDofs(), p);
  return forceModelAssembler->compute_E(std::span<const double>(p.data(), static_cast<std::size_t>(p.size())), state);
}

void DeformationEnergyOperator::compute_dE_dp(
  ES::ConstRefVecXd displacement, MaterialStateView state, ES::RefVecXd grad) const
{
  ES::VXd &p = absolutePositionScratch_;
  fillAbsolutePositions(displacement, getRestDofs(), p);
  forceModelAssembler->compute_dE_dp(
    std::span<const double>(p.data(), static_cast<std::size_t>(p.size())), state,
    grad);
}

void DeformationEnergyOperator::compute_dE_de(
  ES::ConstRefVecXd displacement, MaterialStateView state, ES::RefVecXd grad) const
{
  ES::VXd &p = absolutePositionScratch_;
  fillAbsolutePositions(displacement, getRestDofs(), p);
  forceModelAssembler->compute_dE_de(
    std::span<const double>(p.data(), static_cast<std::size_t>(p.size())), state,
    grad);
}

void DeformationEnergyOperator::computePlasticMaterialVJP(
  EigenSupport::ConstRefVecXd displacement,
  MaterialStateView state,
  EigenSupport::ConstRefVecXd adjoint,
  EigenSupport::RefVecXd output) const
{
  ES::VXd &p = absolutePositionScratch_;
  fillAbsolutePositions(displacement, getRestDofs(), p);
  forceModelAssembler->computePlasticMaterialVJP(
    std::span<const double>(p.data(), static_cast<std::size_t>(p.size())),
    std::span<const double>(adjoint.data(), static_cast<std::size_t>(adjoint.size())),
    state,
    std::span<double>(output.data(), static_cast<std::size_t>(output.size())));
}

void DeformationEnergyOperator::computeElasticMaterialVJP(
  EigenSupport::ConstRefVecXd displacement,
  MaterialStateView state,
  EigenSupport::ConstRefVecXd adjoint,
  EigenSupport::RefVecXd output) const
{
  ES::VXd &p = absolutePositionScratch_;
  fillAbsolutePositions(displacement, getRestDofs(), p);
  forceModelAssembler->computeElasticMaterialVJP(
    std::span<const double>(p.data(), static_cast<std::size_t>(p.size())),
    std::span<const double>(adjoint.data(), static_cast<std::size_t>(adjoint.size())),
    state,
    std::span<double>(output.data(), static_cast<std::size_t>(output.size())));
}

void DeformationEnergyOperator::computeVonMisesStresses(
  ES::ConstRefVecXd displacement, MaterialStateView state,
  ES::RefVecXd elementStresses) const
{
  ES::VXd &p = absolutePositionScratch_;
  fillAbsolutePositions(displacement, getRestDofs(), p);
  forceModelAssembler->computeVonMisesStresses(
    std::span<const double>(p.data(), static_cast<std::size_t>(p.size())), state,
    std::span<double>(elementStresses.data(), static_cast<std::size_t>(elementStresses.size())));
}

void DeformationEnergyOperator::computeMaxStrains(
  ES::ConstRefVecXd displacement, MaterialStateView state,
  ES::RefVecXd elementStrains) const
{
  ES::VXd &p = absolutePositionScratch_;
  fillAbsolutePositions(displacement, getRestDofs(), p);
  forceModelAssembler->computeMaxStrains(
    std::span<const double>(p.data(), static_cast<std::size_t>(p.size())), state,
    std::span<double>(elementStrains.data(), static_cast<std::size_t>(elementStrains.size())));
}

void DeformationEnergyOperator::gradient(
  EigenSupport::ConstRefVecXd x,
  MaterialStateView state,
  EigenSupport::RefVecXd grad) const
{
  Profiling::ScopedProfileSection scopedProfile("material.gradient");
  ES::VXd &p = absolutePositionScratch_;
  fillAbsolutePositions(x, getRestDofs(), p);
  forceModelAssembler->compute_dE_dx(std::span<const double>(p.data(), static_cast<std::size_t>(p.size())), state,
    grad);
}

void DeformationEnergyOperator::hessianInPlace(
  EigenSupport::ConstRefVecXd x,
  MaterialStateView state,
  EigenSupport::SpMatD &hess) const
{
  Profiling::ScopedProfileSection scopedProfile("material.hessian");
  ES::VXd &p = absolutePositionScratch_;
  fillAbsolutePositions(x, getRestDofs(), p);
  forceModelAssembler->compute_d2E_dx2(std::span<const double>(p.data(), static_cast<std::size_t>(p.size())), state, hess);
}

void DeformationEnergyOperator::hessianAlloc(EigenSupport::SpMatD &hess) const
{
  hess = forceModelAssembler->getHessianTemplate();
}

NonlinearOptimization::StepConstraint DeformationEnergyOperator::computeMaxStepLimit(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx, StepConstraintSink *sink) const
{
  Profiling::ScopedProfileSection scopedProfile("material.max_step");
  if (!enableMaterialMaxStep_) {
    return NonlinearOptimization::StepConstraint{};
  }

  const int numDOFs = getNumDOFs();
  if (x.size() != numDOFs || dx.size() != numDOFs) {
    throw std::invalid_argument(
      "DeformationEnergyOperator::computeMaxStepLimit: local state size does not match the energy DOF count.");
  }

  if (dx.size() == 0 || dx.squaredNorm() == 0.0) {
    return NonlinearOptimization::StepConstraint{};
  }

  ES::VXd &absolutePositions = absolutePositionScratch_;
  fillAbsolutePositions(x, getRestDofs(), absolutePositions);
  const auto observation = forceModelAssembler->computeMaxStepObservation(std::span<const double>(absolutePositions.data(), static_cast<std::size_t>(absolutePositions.size())), std::span<const double>(dx.data(), static_cast<std::size_t>(dx.size())));
  const double maxStepSize = observation.alpha;

  if (maxStepSize < 1.0) {
    const auto meshType = forceModelAssembler->meshType();

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

  NonlinearOptimization::StepConstraint c{ NonlinearOptimization::StepSource::Material, maxStepSize };
  if (sink)
    sink->report(c);
  return c;
}
