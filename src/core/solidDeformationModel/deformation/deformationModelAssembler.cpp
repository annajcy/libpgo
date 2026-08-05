/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "deformation/deformationModelAssembler.h"
#include "deformation/materialMaxStepPolynomialUtils.h"
#include "deformation/deformationModelManager.h"
#include "simulation/simulationMesh.h"
#include "deformation/deformationModel.h"
#include "material/elastic/elasticModel.h"
#include "material/plastic/plasticModel.h"
#include "material/runtime/materialState.h"

#include "pgoLogging.h"
#include "EigenSupport.h"
#include "fmtEigen.h"
#include "processMemory.h"

#include <algorithm>
#include <atomic>
#include <set>
#include <stdexcept>
#include <string>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

using namespace pgo::SolidDeformationModel;

namespace ES = pgo::EigenSupport;

namespace
{
void logMemoryCheckpoint(const char *stage)
{
  constexpr double bytesPerMiB = 1024.0 * 1024.0;
  const pgo::Profiling::ProcessMemoryUsage usage =
    pgo::Profiling::recordProcessMemoryProfileCounters(stage);
  SPDLOG_LOGGER_INFO(pgo::Logging::lgr(),
    "Process memory checkpoint stage={} currentMiB={:.2f} peakMiB={:.2f}",
    stage, usage.residentBytes / bytesPerMiB, usage.peakResidentBytes / bytesPerMiB);
}

void sanityCheckValues(std::span<double> values, Eigen::Index count, const char *label)
{
  for (Eigen::Index i = 0; i < count; i++) {
    int fpclass = std::fpclassify(values[i]);
    if (fpclass == FP_INFINITE || fpclass == FP_NAN) {
      SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Encounter weird {} numbers at {}: {}", label, i, values[i]);
      throw std::logic_error("Encounter weird numbers.");
    }
    else if (fpclass == FP_SUBNORMAL) {
      values[i] = 0;
    }
  }
}

void warnIllegalInitialState(pgo::SolidDeformationModel::SimulationMeshType meshType, int elementId, int locationId, double phi0, double eps)
{
  if (locationId >= 0) {
    SPDLOG_LOGGER_WARN(pgo::Logging::lgr(),
      "material max step encountered illegal initial state on meshType={} element={} location={} : phi(0)={} <= eps={}. Returning recovery clamp {}.",
      meshTypeName(meshType), elementId, locationId, phi0, eps, pgo::SolidDeformationModel::kMaterialMaxStepMinClamp);
  }
  else {
    SPDLOG_LOGGER_WARN(pgo::Logging::lgr(),
      "material max step encountered illegal initial state on meshType={} element={} : phi(0)={} <= eps={}. Returning recovery clamp {}.",
      meshTypeName(meshType), elementId, phi0, eps, pgo::SolidDeformationModel::kMaterialMaxStepMinClamp);
  }
}

void fillLocalParamDerivative(
  const OptimizableParameterField &block,
  const MaterialStateView &state,
  int ele,
  int quadratureId,
  std::span<double> localDofValues,
  ES::RefMatXd derivOut)
{
  const auto &layout = block.layout();
  const auto &mapping = block.mapping();
  const int numChannels = mapping.numChannels();
  const int numLocalParameters = layout.numLocalParameters();
  if (derivOut.rows() != numChannels || derivOut.cols() != numLocalParameters)
    throw std::invalid_argument("Material channel Jacobian output has unexpected shape.");
  if (numLocalParameters > 0) {
    layout.gather(
      ele, state.values(block),
      localDofValues.first(static_cast<std::size_t>(numLocalParameters)));
  }
  if (numChannels > 0 && numLocalParameters > 0) {
    mapping.evaluateJacobian(
      ele, quadratureId,
      localDofValues.first(static_cast<std::size_t>(numLocalParameters)),
      derivOut);
  }
}

}  // namespace

DeformationModelAssembler::DeformationModelAssembler(
  std::shared_ptr<const DeformationModelManager> dm,
  const Formulation &formulation,
  std::shared_ptr<const OptimizableParameterField> elasticField,
  std::shared_ptr<const OptimizableParameterField> plasticField,
  std::span<const double> elementWeights_):
  deformationModelManager(std::move(dm)),
  dofLayout(formulation.createDofLayout(deformationModelManager->getMesh())),
  restDofs_(formulation.buildGlobalRestDofs(deformationModelManager->getMesh())),
  elasticField_(std::move(elasticField)),
  plasticField_(std::move(plasticField))
{
  if (!elasticField_ || !plasticField_)
    throw std::invalid_argument(
      "DeformationModelAssembler requires elastic and plastic parameter fields.");
  nele = deformationModelManager->getMesh().getNumElements();
  neleVtx = deformationModelManager->getMesh().getNumElementVertices();
  localDOFs = dofLayout->numLocalDofs(0);
  numDOFs = dofLayout->numGlobalDofs();

  numElasticParams_ = deformationModelManager->getNumElasticParameters();
  numPlasticParams_ = deformationModelManager->getNumPlasticParameters();
  const auto &elasticBlock = *elasticField_;
  const auto &plasticBlock = *plasticField_;
  if (elasticBlock.mapping().numChannels() != numElasticParams_)
    throw std::invalid_argument("DeformationModelAssembler elastic channel count does not match the material model.");
  if (plasticBlock.mapping().numChannels() != numPlasticParams_)
    throw std::invalid_argument("DeformationModelAssembler plastic channel count does not match the material model.");
  if (elasticBlock.layout().numElements() != nele ||
    plasticBlock.layout().numElements() != nele)
    throw std::invalid_argument("DeformationModelAssembler parameter layout element count does not match the mesh.");

  const auto *elasticParamLayout = &elasticBlock.layout();
  const auto *plasticParamLayout = &plasticBlock.layout();
  numElasticLocalParams_ = elasticParamLayout->numLocalParameters();
  numPlasticLocalParams_ = plasticParamLayout->numLocalParameters();

  if (!elementWeights_.empty()) {
    if (elementWeights_.size() != static_cast<std::size_t>(nele))
      throw std::invalid_argument("DeformationModelAssembler element weight count does not match the mesh.");
    elementWeights.assign(elementWeights_.begin(), elementWeights_.end());
  }
  else {
    elementWeights.assign(nele, 1);
  }

  int maxMaterialLocations = 0;
  for (int i = 0; i < nele; i++) {
    femModels.emplace_back(deformationModelManager->getDeformationModel(i));
    maxMaterialLocations = std::max(maxMaterialLocations, femModels.back().get().getNumMaterialLocations());
  }

  const int maxMaterialParams = std::max(numElasticParams_, numPlasticParams_);
  const int maxLocalParams = std::max(numElasticLocalParams_, numPlasticLocalParams_);
  elementWorkspaces_.reserve(femModels.size());
  for (const auto &model : femModels)
    elementWorkspaces_.emplace_back(
      localDOFs, maxMaterialLocations, maxMaterialParams, maxLocalParams,
      model.get());
  for (int ele = 0; ele < nele; ele++)
    dofLayout->getDofGroups(ele, elementWorkspaces_[ele].groups);
  logMemoryCheckpoint("assembler.after_element_cache_setup");

  SPDLOG_LOGGER_INFO(Logging::lgr(), "Assembler parameter channels:{},{} local:{},{}",
    numElasticParams_, numPlasticParams_, numElasticLocalParams_, numPlasticLocalParams_);

  // Hessian template.
  std::set<HessianBlockKey> hessianBlocks;
  dofLayout->collectHessianBlockPairs(nele, hessianBlocks);
  logMemoryCheckpoint("assembler.after_unique_hessian_block_collection");

  buildCompressedHessianTemplate(numDOFs, hessianBlocks, KTemplate);
  logMemoryCheckpoint("assembler.after_compressed_hessian_template");

  dofLayout->buildAllHessianBlockOffsets(nele, KTemplate, elementKBlockOffsets);
  logMemoryCheckpoint("assembler.after_element_block_offsets");

  // NOTE (tricubic Hermite seam): material-parameter templates remain scalar-index based because
  // they are not the displacement Hessian path that needs grouped offsets.
  std::vector<ES::TripletD> entries;

  // Displacement-elastic Hessian template.
  const int numElasticGlobalParams = elasticParamLayout ? elasticParamLayout->numGlobalParameters() : 0;
  if (numElasticParams_ > 0 && numElasticLocalParams_ > 0 && elasticParamLayout) {
    buildMixedSparsityTemplate(
      numElasticLocalParams_, numElasticGlobalParams,
      [elasticParamLayout](int ele, int ep) { return elasticParamLayout->globalParameter(ele, ep); },
      d2E_dudeTemplate, element_d2E_dude_InverseIndices, entries);
  }
  else {
    d2E_dudeTemplate.resize(numDOFs, 0);
  }

  // Displacement-plastic Hessian template.
  const int numPlasticGlobalParams = plasticParamLayout ? plasticParamLayout->numGlobalParameters() : 0;
  if (numPlasticParams_ > 0 && numPlasticLocalParams_ > 0 && plasticParamLayout) {
    buildMixedSparsityTemplate(
      numPlasticLocalParams_, numPlasticGlobalParams,
      [plasticParamLayout](int ele, int pp) { return plasticParamLayout->globalParameter(ele, pp); },
      d2E_dudpTemplate, element_d2E_dudp_InverseIndices, entries);
  }
  else {
    d2E_dudpTemplate.resize(numDOFs, 0);
  }

  // d²E/dp² (plastic-only) template.
  entries.clear();
  if (numPlasticParams_ > 0 && numPlasticLocalParams_ > 0 && plasticParamLayout) {
    for (int ele = 0; ele < nele; ele++) {
      for (int pi = 0; pi < numPlasticLocalParams_; pi++) {
        const int globalRow = plasticParamLayout->globalParameter(ele, pi);
        for (int pj = 0; pj < numPlasticLocalParams_; pj++) {
          const int globalCol = plasticParamLayout->globalParameter(ele, pj);
          entries.emplace_back(globalRow, globalCol, 1.0);
        }
      }
    }

    d2E_dp2Template.resize(numPlasticGlobalParams, numPlasticGlobalParams);
    d2E_dp2Template.setFromTriplets(entries.begin(), entries.end());
  }
  else {
    d2E_dp2Template.resize(0, 0);
  }

  element_d2E_dp2_InverseIndices.resize(nele);
  if (numPlasticParams_ > 0 && numPlasticLocalParams_ > 0 && plasticParamLayout) {
    for (int ele = 0; ele < nele; ele++) {
      DynamicIndexMatrix idxM(numPlasticLocalParams_, numPlasticLocalParams_);
      idxM.setConstant(-1);

      for (int pi = 0; pi < numPlasticLocalParams_; pi++) {
        const int globalRow = plasticParamLayout->globalParameter(ele, pi);
        for (int pj = 0; pj < numPlasticLocalParams_; pj++) {
          const int globalCol = plasticParamLayout->globalParameter(ele, pj);
          idxM(pi, pj) = ES::findEntryOffset(
            d2E_dp2Template, globalRow, globalCol);
        }
      }

      element_d2E_dp2_InverseIndices[ele] = idxM;
    }
  }

  // d²E/de² (elastic-only) template.
  entries.clear();
  if (numElasticParams_ > 0 && numElasticLocalParams_ > 0 && elasticParamLayout) {
    for (int ele = 0; ele < nele; ele++) {
      for (int pi = 0; pi < numElasticLocalParams_; pi++) {
        const int globalRow = elasticParamLayout->globalParameter(ele, pi);
        for (int pj = 0; pj < numElasticLocalParams_; pj++) {
          const int globalCol = elasticParamLayout->globalParameter(ele, pj);
          entries.emplace_back(globalRow, globalCol, 1.0);
        }
      }
    }

    d2E_de2Template.resize(numElasticGlobalParams, numElasticGlobalParams);
    d2E_de2Template.setFromTriplets(entries.begin(), entries.end());
  }
  else {
    d2E_de2Template.resize(0, 0);
  }

  element_d2E_de2_InverseIndices.resize(nele);
  if (numElasticParams_ > 0 && numElasticLocalParams_ > 0 && elasticParamLayout) {
    for (int ele = 0; ele < nele; ele++) {
      DynamicIndexMatrix idxM(numElasticLocalParams_, numElasticLocalParams_);
      idxM.setConstant(-1);

      for (int pi = 0; pi < numElasticLocalParams_; pi++) {
        const int globalRow = elasticParamLayout->globalParameter(ele, pi);
        for (int pj = 0; pj < numElasticLocalParams_; pj++) {
          const int globalCol = elasticParamLayout->globalParameter(ele, pj);
          idxM(pi, pj) = ES::findEntryOffset(
            d2E_de2Template, globalRow, globalCol);
        }
      }

      element_d2E_de2_InverseIndices[ele] = idxM;
    }
  }

  // d²E/dp de (plastic rows, elastic columns).
  entries.clear();
  if (numPlasticParams_ > 0 && numPlasticLocalParams_ > 0 && plasticParamLayout &&
    numElasticParams_ > 0 && numElasticLocalParams_ > 0 && elasticParamLayout) {
    for (int ele = 0; ele < nele; ele++) {
      for (int pi = 0; pi < numPlasticLocalParams_; pi++) {
        const int globalRow = plasticParamLayout->globalParameter(ele, pi);
        for (int ej = 0; ej < numElasticLocalParams_; ej++) {
          const int globalCol = elasticParamLayout->globalParameter(ele, ej);
          entries.emplace_back(globalRow, globalCol, 1.0);
        }
      }
    }

    d2E_dpdeTemplate.resize(numPlasticGlobalParams, numElasticGlobalParams);
    d2E_dpdeTemplate.setFromTriplets(entries.begin(), entries.end());
  }
  else {
    d2E_dpdeTemplate.resize(0, 0);
  }

  element_d2E_dpde_InverseIndices.resize(nele);
  if (numPlasticParams_ > 0 && numPlasticLocalParams_ > 0 && plasticParamLayout &&
    numElasticParams_ > 0 && numElasticLocalParams_ > 0 && elasticParamLayout) {
    for (int ele = 0; ele < nele; ele++) {
      DynamicIndexMatrix idxM(numPlasticLocalParams_, numElasticLocalParams_);
      idxM.setConstant(-1);

      for (int pi = 0; pi < numPlasticLocalParams_; pi++) {
        const int globalRow = plasticParamLayout->globalParameter(ele, pi);
        for (int ej = 0; ej < numElasticLocalParams_; ej++) {
          const int globalCol = elasticParamLayout->globalParameter(ele, ej);
          idxM(pi, ej) = ES::findEntryOffset(
            d2E_dpdeTemplate, globalRow, globalCol);
        }
      }

      element_d2E_dpde_InverseIndices[ele] = idxM;
    }
  }
}

DeformationModelAssembler::~DeformationModelAssembler() = default;

void DeformationModelAssembler::validateMaterialState(
  const MaterialStateView &state) const
{
  if (state.empty())
    throw std::invalid_argument("DeformationModelAssembler requires a non-empty material state.");
  if (!state.elasticField().sharesStateWith(*elasticField_) ||
    !state.plasticField().sharesStateWith(*plasticField_))
    throw std::invalid_argument(
      "MaterialStateView belongs to different parameter fields.");
  if (state.elasticValues().size() != static_cast<std::size_t>(getNumElasticGlobalParams()))
    throw std::invalid_argument("MaterialStateView elastic value count does not match the assembler.");
  if (state.plasticValues().size() != static_cast<std::size_t>(getNumPlasticGlobalParams()))
    throw std::invalid_argument("MaterialStateView plastic value count does not match the assembler.");
}

void DeformationModelAssembler::validatePositionSpan(
  std::span<const double> x, const char *label) const
{
  if (x.size() != static_cast<std::size_t>(numDOFs))
    throw std::invalid_argument(
      std::string("DeformationModelAssembler ") + label + " has unexpected size.");
}

DeformationModelEvaluator &DeformationModelAssembler::gatherAndPrepare(
  int ele, std::span<const double> x, const MaterialStateView &state,
  DeformationModelAssemblerElementWorkspace &scratch) const
{
  std::fill(scratch.localPosition.data(), scratch.localPosition.data() + localDOFs, 0.0);
  for (const DofGroup &group : scratch.groups) {
    for (int i = 0; i < group.size; i++)
      scratch.localPosition[group.localStart + i] = x[group.globalDof(i)];
  }
  const DeformationModel &fem = femModels[ele].get();
  DeformationModelEvaluator &evaluator = scratch.evaluator();
  const int numMaterialLocations = fem.getNumMaterialLocations();
  state.evaluateElement(
    *elasticField_, ele, numMaterialLocations,
    std::span<double>(scratch.localParamValues.data(), scratch.localParamValues.size()),
    std::span<double>(scratch.elasticParamValues.data(), scratch.elasticParamValues.size()));
  state.evaluateElement(
    *plasticField_, ele, numMaterialLocations,
    std::span<double>(scratch.localParamValues.data(), scratch.localParamValues.size()),
    std::span<double>(scratch.plasticParamValues.data(), scratch.plasticParamValues.size()));

  evaluator.prepare(std::span<const double>(scratch.localPosition.data(), localDOFs),
    std::span<const double>(scratch.elasticParamValues.data(),
      static_cast<std::size_t>(numMaterialLocations * numElasticParams_)),
    std::span<const double>(scratch.plasticParamValues.data(),
      static_cast<std::size_t>(numMaterialLocations * numPlasticParams_)));
  return evaluator;
}

double DeformationModelAssembler::compute_E(
  std::span<const double> x, MaterialStateView state) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);

  auto localEnergyFunc = [this, x, &state](int ele) {
    auto &scratch = elementWorkspaces_[ele];
    scratch.energy = 0.0;
    if (elementWeights[ele] == 0)
      return;

    DeformationModelEvaluator &evaluator = gatherAndPrepare(ele, x, state, scratch);
    double energy = evaluator.compute_E();

    scratch.energy = energy * elementWeights[ele];
  };

  tbb::parallel_for(0, nele, localEnergyFunc);

  double energyAll = 0;
  for (int ele = 0; ele < nele; ele++)
    energyAll += elementWorkspaces_[ele].energy;

  return energyAll;
}

DeformationModelAssembler::MaterialMaxStepObservation DeformationModelAssembler::computeMaxStepObservation(std::span<const double> x, std::span<const double> dx) const
{
  validatePositionSpan(x, "position vector");
  validatePositionSpan(dx, "direction vector");
  MaterialMaxStepObservation observation;
  const SimulationMeshType meshType = deformationModelManager->getMesh().getElementType();

  for (int ele = 0; ele < nele; ele++) {
    if (elementWeights[ele] == 0) {
      continue;
    }

    auto &scratch = elementWorkspaces_[ele];
    std::fill(scratch.localPosition.data(), scratch.localPosition.data() + localDOFs, 0.0);
    std::fill(scratch.localDirection.data(), scratch.localDirection.data() + localDOFs, 0.0);
    for (const DofGroup &group : scratch.groups) {
      for (int i = 0; i < group.size; i++) {
        scratch.localPosition[group.localStart + i] = x[group.globalDof(i)];
        scratch.localDirection[group.localStart + i] = dx[group.globalDof(i)];
      }
    }

    const DeformationModel::LocalMaxStepResult localResult =
      femModels[ele].get().computeLocalMaxStepSize(
        std::span<const double>(scratch.localPosition.data(), scratch.localPosition.size()),
        std::span<const double>(scratch.localDirection.data(), scratch.localDirection.size()));
    if (localResult.alpha < observation.alpha) {
      observation.alpha = localResult.alpha;
      observation.limitingElementId = ele;
      observation.limitingLocationId = localResult.locationId;
    }
    if (localResult.illegalInitialState) {
      observation.hasIllegalInitialState = true;
      warnIllegalInitialState(meshType, ele, localResult.locationId, localResult.phi0, localResult.eps);
    }

    if (observation.alpha <= kMaterialMaxStepMinClamp) {
      break;
    }
  }

  return observation;
}

double DeformationModelAssembler::computeMaxStepSize(std::span<const double> x, std::span<const double> dx) const
{
  return computeMaxStepObservation(x, dx).alpha;
}

void DeformationModelAssembler::compute_dE_dx(
  std::span<const double> x, MaterialStateView state, ES::RefVecXd grad) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  if (grad.size() != numDOFs)
    throw std::invalid_argument("DeformationModelAssembler gradient has unexpected size.");
  grad.setZero();
  auto localGradFunc = [this, x, &state, &grad](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elementWorkspaces_[ele];
    DeformationModelEvaluator &evaluator = gatherAndPrepare(ele, x, state, scratch);

    evaluator.compute_dE_dx(scratch.localGradient);
    scratch.localGradient *= elementWeights[ele];

    if (enableSanityCheck) {
      for (int i = 0; i < localDOFs; i++) {
        if (std::isfinite(scratch.localGradient[i]) == false) {
          SPDLOG_LOGGER_ERROR(Logging::lgr(), "Ele: {}", ele);
          SPDLOG_LOGGER_ERROR(Logging::lgr(), "Encounter weird numbers.\nGrad:\n{}\n;x:{}\n",
            scratch.localGradient, scratch.localPosition);
        }
      }
    }

    for (const DofGroup &group : scratch.groups) {
      for (int i = 0; i < group.size; i++) {
        std::atomic_ref<double> atomicGrad(grad[group.globalDof(i)]);
        atomicGrad.fetch_add(scratch.localGradient[group.localStart + i]);
      }
    }
  };

  tbb::parallel_for(0, nele, localGradFunc);

  if (enableSanityCheck)
    sanityCheckValues(std::span<double>(grad.data(), static_cast<std::size_t>(grad.size())),
      numDOFs, "gradient");
}

void DeformationModelAssembler::compute_d2E_dx2(
  std::span<const double> x, MaterialStateView state, EigenSupport::SpMatD &hess) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  auto localHessFunc = [this, x, &state, &hess](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elementWorkspaces_[ele];
    DeformationModelEvaluator &evaluator = gatherAndPrepare(ele, x, state, scratch);

    ES::Mp<ES::MXd> localK(scratch.localMatrixData.data(), localDOFs, localDOFs);
    evaluator.compute_d2E_dx2(localK);

    localK *= elementWeights[ele];

    const auto &blocks = elementKBlockOffsets[ele];
    for (const auto &block : blocks) {
      for (int localCol = 0; localCol < block.colSize; localCol++) {
        const int srcCol = block.colLocalStart + localCol;
        for (int localRow = 0; localRow < block.rowSize; localRow++) {
          const int srcRow = block.rowLocalStart + localRow;
          const std::ptrdiff_t offset = block.offset(localRow, localCol);
          std::atomic_ref<double> hessRef(hess.valuePtr()[offset]);
          hessRef.fetch_add(localK(srcRow, srcCol));
        }
      }
    }
  };

  tbb::parallel_for(0, nele, localHessFunc);

  if (enableSanityCheck)
    sanityCheckValues(std::span<double>(hess.valuePtr(), hess.nonZeros()), hess.nonZeros(), "Hessian");
}

int DeformationModelAssembler::getNumElasticGlobalParams() const
{
  return elasticField_->layout().numGlobalParameters();
}

int DeformationModelAssembler::getNumPlasticGlobalParams() const
{
  return plasticField_->layout().numGlobalParameters();
}

void DeformationModelAssembler::compute_dE_dp(
  std::span<const double> x, MaterialStateView state, ES::RefVecXd grad) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  const int numPlasticGlobalParams = getNumPlasticGlobalParams();
  if (grad.size() != numPlasticGlobalParams)
    throw std::invalid_argument("DeformationModelAssembler plastic gradient has unexpected size.");
  grad.setZero();

  if (numPlasticParams_ == 0 || numPlasticLocalParams_ == 0 || numPlasticGlobalParams == 0)
    return;

  const auto &plasticBlock = *plasticField_;
  const auto *plasticParamLayout = &plasticBlock.layout();

  auto localGradFunc = [this, x, &state, &grad, plasticParamLayout, &plasticBlock](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elementWorkspaces_[ele];
    DeformationModelEvaluator &evaluator = gatherAndPrepare(ele, x, state, scratch);

    Eigen::Map<ES::VXd> rawGrad(
      scratch.rawParamGradient.data(), numPlasticParams_);
    auto dParamDLocal = scratch.paramDerivativeData.block(
      0, 0, numPlasticParams_, numPlasticLocalParams_);
    auto localGrad =
      scratch.localParamGradient.head(numPlasticLocalParams_);
    localGrad.setZero();
    for (int q = 0; q < femModels[ele].get().getNumMaterialLocations(); q++) {
      rawGrad.setZero();
      evaluator.compute_dE_dp(rawGrad, q);
      fillLocalParamDerivative(
        plasticBlock, state, ele, q,
        std::span<double>(scratch.localParamValues.data(), scratch.localParamValues.size()),
        dParamDLocal);
      localGrad.noalias() += dParamDLocal.transpose() * rawGrad;
    }
    localGrad *= elementWeights[ele];

    for (int pi = 0; pi < numPlasticLocalParams_; pi++) {
      const int globalRow = plasticParamLayout->globalParameter(ele, pi);
      std::atomic_ref<double> gradRef(grad[globalRow]);
      gradRef.fetch_add(scratch.localParamGradient[pi]);
    }
  };

  tbb::parallel_for(0, nele, localGradFunc);

  if (enableSanityCheck)
    sanityCheckValues(std::span<double>(grad.data(), static_cast<std::size_t>(grad.size())),
      numPlasticGlobalParams, "plastic gradient");
}

void DeformationModelAssembler::compute_d2E_dp2(
  std::span<const double> x, MaterialStateView state, EigenSupport::SpMatD &hess) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  if (hess.rows() != d2E_dp2Template.rows() ||
    hess.cols() != d2E_dp2Template.cols() ||
    hess.nonZeros() != d2E_dp2Template.nonZeros()) {
    hess = d2E_dp2Template;
  }

  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  const int numPlasticGlobalParams = getNumPlasticGlobalParams();
  if (numPlasticParams_ == 0 || numPlasticLocalParams_ == 0 || numPlasticGlobalParams == 0)
    return;

  const auto &plasticBlock = *plasticField_;
  auto localHessFunc = [this, x, &state, &hess, &plasticBlock](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elementWorkspaces_[ele];
    DeformationModelEvaluator &evaluator = gatherAndPrepare(ele, x, state, scratch);

    const ES::Mp<ES::MXd> rawH(scratch.localMatrixData.data(), numPlasticParams_, numPlasticParams_);
    auto dParamDLocal = scratch.paramDerivativeData.block(
      0, 0, numPlasticParams_, numPlasticLocalParams_);
    auto paramWork = scratch.paramWorkMatrix.block(0, 0, numPlasticParams_, numPlasticLocalParams_);
    auto localH = scratch.localParamHessian.block(0, 0, numPlasticLocalParams_, numPlasticLocalParams_);
    localH.setZero();
    for (int q = 0; q < femModels[ele].get().getNumMaterialLocations(); q++) {
      evaluator.compute_d2E_dp2(ES::Mp<ES::MXd>(scratch.localMatrixData.data(), numPlasticParams_, numPlasticParams_), q);
      fillLocalParamDerivative(
        plasticBlock, state, ele, q,
        std::span<double>(scratch.localParamValues.data(), scratch.localParamValues.size()),
        dParamDLocal);
      paramWork.noalias() = rawH * dParamDLocal;
      localH.noalias() += dParamDLocal.transpose() * paramWork;
      if (!plasticBlock.mapping().isAffine()) {
        Eigen::Map<ES::VXd> rawGrad(
          scratch.rawParamGradient.data(), numPlasticParams_);
        rawGrad.setZero();
        evaluator.compute_dE_dp(rawGrad, q);
        const std::span<EigenSupport::MXd> evaluatorHessians =
          scratch.preparePlasticParamEvaluatorHessians(
            numPlasticParams_, numPlasticLocalParams_);
        plasticBlock.mapping().evaluateHessians(
          ele, q,
          std::span<const double>(
            scratch.localParamValues.data(), numPlasticLocalParams_),
          evaluatorHessians);
        for (int channel = 0; channel < numPlasticParams_; channel++) {
          localH.noalias() += rawGrad[channel] *
            evaluatorHessians[static_cast<std::size_t>(channel)];
        }
      }
    }
    localH *= elementWeights[ele];

    const auto &idxM = element_d2E_dp2_InverseIndices[ele];
    for (int localRow = 0; localRow < numPlasticLocalParams_; localRow++) {
      for (int localCol = 0; localCol < numPlasticLocalParams_; localCol++) {
        std::ptrdiff_t offset = idxM(localRow, localCol);
        if (offset >= 0) {
          std::atomic_ref<double> hessRef(hess.valuePtr()[offset]);
          hessRef.fetch_add(localH(localRow, localCol));
        }
      }
    }
  };

  tbb::parallel_for(0, nele, localHessFunc);

  if (enableSanityCheck)
    sanityCheckValues(std::span<double>(hess.valuePtr(), hess.nonZeros()), hess.nonZeros(), "plastic Hessian");
}

void DeformationModelAssembler::compute_dE_de(
  std::span<const double> x, MaterialStateView state, ES::RefVecXd grad) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  const int numElasticGlobalParams = getNumElasticGlobalParams();
  if (grad.size() != numElasticGlobalParams)
    throw std::invalid_argument("DeformationModelAssembler elastic gradient has unexpected size.");
  grad.setZero();

  if (numElasticParams_ == 0 || numElasticLocalParams_ == 0 || numElasticGlobalParams == 0)
    return;

  const auto &elasticBlock = *elasticField_;
  const auto *elasticParamLayout = &elasticBlock.layout();

  auto localGradFunc = [this, x, &state, &grad, elasticParamLayout, &elasticBlock](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elementWorkspaces_[ele];
    DeformationModelEvaluator &evaluator = gatherAndPrepare(ele, x, state, scratch);

    Eigen::Map<ES::VXd> rawGrad(
      scratch.rawParamGradient.data(), numElasticParams_);
    auto dParamDLocal = scratch.paramDerivativeData.block(
      0, 0, numElasticParams_, numElasticLocalParams_);
    auto localGrad =
      scratch.localParamGradient.head(numElasticLocalParams_);
    localGrad.setZero();
    for (int q = 0; q < femModels[ele].get().getNumMaterialLocations(); q++) {
      rawGrad.setZero();
      evaluator.compute_dE_de(rawGrad, q);
      fillLocalParamDerivative(
        elasticBlock, state, ele, q,
        std::span<double>(scratch.localParamValues.data(), scratch.localParamValues.size()),
        dParamDLocal);
      localGrad.noalias() += dParamDLocal.transpose() * rawGrad;
    }
    localGrad *= elementWeights[ele];

    for (int pi = 0; pi < numElasticLocalParams_; pi++) {
      const int globalRow = elasticParamLayout->globalParameter(ele, pi);
      std::atomic_ref<double> gradRef(grad[globalRow]);
      gradRef.fetch_add(scratch.localParamGradient[pi]);
    }
  };

  tbb::parallel_for(0, nele, localGradFunc);

  if (enableSanityCheck)
    sanityCheckValues(std::span<double>(grad.data(), static_cast<std::size_t>(grad.size())),
      numElasticGlobalParams, "elastic gradient");
}

void DeformationModelAssembler::compute_d2E_de2(
  std::span<const double> x, MaterialStateView state, EigenSupport::SpMatD &hess) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  if (hess.rows() != d2E_de2Template.rows() ||
    hess.cols() != d2E_de2Template.cols() ||
    hess.nonZeros() != d2E_de2Template.nonZeros()) {
    hess = d2E_de2Template;
  }

  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  const int numElasticGlobalParams = getNumElasticGlobalParams();
  if (numElasticParams_ == 0 || numElasticLocalParams_ == 0 || numElasticGlobalParams == 0)
    return;

  const auto &elasticBlock = *elasticField_;
  auto localHessFunc = [this, x, &state, &hess, &elasticBlock](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elementWorkspaces_[ele];
    DeformationModelEvaluator &evaluator = gatherAndPrepare(ele, x, state, scratch);

    const ES::Mp<ES::MXd> rawH(scratch.localMatrixData.data(), numElasticParams_, numElasticParams_);
    auto dParamDLocal = scratch.paramDerivativeData.block(
      0, 0, numElasticParams_, numElasticLocalParams_);
    auto paramWork = scratch.paramWorkMatrix.block(0, 0, numElasticParams_, numElasticLocalParams_);
    auto localH = scratch.localParamHessian.block(0, 0, numElasticLocalParams_, numElasticLocalParams_);
    localH.setZero();
    for (int q = 0; q < femModels[ele].get().getNumMaterialLocations(); q++) {
      evaluator.compute_d2E_de2(ES::Mp<ES::MXd>(scratch.localMatrixData.data(), numElasticParams_, numElasticParams_), q);
      fillLocalParamDerivative(
        elasticBlock, state, ele, q,
        std::span<double>(scratch.localParamValues.data(), scratch.localParamValues.size()),
        dParamDLocal);
      paramWork.noalias() = rawH * dParamDLocal;
      localH.noalias() += dParamDLocal.transpose() * paramWork;
      if (!elasticBlock.mapping().isAffine()) {
        Eigen::Map<ES::VXd> rawGrad(
          scratch.rawParamGradient.data(), numElasticParams_);
        rawGrad.setZero();
        evaluator.compute_dE_de(rawGrad, q);
        const std::span<EigenSupport::MXd> evaluatorHessians =
          scratch.prepareElasticParamEvaluatorHessians(
            numElasticParams_, numElasticLocalParams_);
        elasticBlock.mapping().evaluateHessians(
          ele, q,
          std::span<const double>(
            scratch.localParamValues.data(), numElasticLocalParams_),
          evaluatorHessians);
        for (int channel = 0; channel < numElasticParams_; channel++) {
          localH.noalias() += rawGrad[channel] *
            evaluatorHessians[static_cast<std::size_t>(channel)];
        }
      }
    }
    localH *= elementWeights[ele];

    const auto &idxM = element_d2E_de2_InverseIndices[ele];
    for (int localRow = 0; localRow < numElasticLocalParams_; localRow++) {
      for (int localCol = 0; localCol < numElasticLocalParams_; localCol++) {
        std::ptrdiff_t offset = idxM(localRow, localCol);
        if (offset >= 0) {
          std::atomic_ref<double> hessRef(hess.valuePtr()[offset]);
          hessRef.fetch_add(localH(localRow, localCol));
        }
      }
    }
  };

  tbb::parallel_for(0, nele, localHessFunc);

  if (enableSanityCheck)
    sanityCheckValues(std::span<double>(hess.valuePtr(), hess.nonZeros()), hess.nonZeros(), "elastic Hessian");
}

void DeformationModelAssembler::compute_d2E_dpde(
  std::span<const double> x, MaterialStateView state, EigenSupport::SpMatD &hess) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  if (hess.rows() != d2E_dpdeTemplate.rows() ||
    hess.cols() != d2E_dpdeTemplate.cols() ||
    hess.nonZeros() != d2E_dpdeTemplate.nonZeros()) {
    hess = d2E_dpdeTemplate;
  }

  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  const int numPlasticGlobalParams = getNumPlasticGlobalParams();
  const int numElasticGlobalParams = getNumElasticGlobalParams();
  if (numPlasticParams_ == 0 || numPlasticLocalParams_ == 0 || numPlasticGlobalParams == 0 ||
    numElasticParams_ == 0 || numElasticLocalParams_ == 0 || numElasticGlobalParams == 0)
    return;

  const auto &plasticBlock = *plasticField_;
  const auto &elasticBlock = *elasticField_;
  auto localHessFunc = [this, x, &state, &hess, &plasticBlock, &elasticBlock](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elementWorkspaces_[ele];
    DeformationModelEvaluator &evaluator = gatherAndPrepare(ele, x, state, scratch);

    const ES::Mp<ES::MXd> rawH(scratch.localMatrixData.data(), numPlasticParams_, numElasticParams_);
    auto dPlasticDLocal = scratch.paramDerivativeData.block(
      0, 0, numPlasticParams_, numPlasticLocalParams_);
    auto dElasticDLocal = scratch.paramDerivativeData2.block(
      0, 0, numElasticParams_, numElasticLocalParams_);

    auto paramWork = scratch.paramWorkMatrix.block(0, 0, numPlasticParams_, numElasticLocalParams_);
    auto localH = scratch.localParamHessian.block(0, 0, numPlasticLocalParams_, numElasticLocalParams_);
    localH.setZero();
    for (int q = 0; q < femModels[ele].get().getNumMaterialLocations(); q++) {
      evaluator.compute_d2E_dpde(ES::Mp<ES::MXd>(scratch.localMatrixData.data(), numPlasticParams_, numElasticParams_), q);
      fillLocalParamDerivative(
        plasticBlock, state, ele, q,
        std::span<double>(scratch.localParamValues.data(), scratch.localParamValues.size()),
        dPlasticDLocal);
      fillLocalParamDerivative(
        elasticBlock, state, ele, q,
        std::span<double>(scratch.localParamValues.data(), scratch.localParamValues.size()),
        dElasticDLocal);
      paramWork.noalias() = rawH * dElasticDLocal;
      localH.noalias() += dPlasticDLocal.transpose() * paramWork;
    }
    localH *= elementWeights[ele];

    const auto &idxM = element_d2E_dpde_InverseIndices[ele];
    for (int localRow = 0; localRow < numPlasticLocalParams_; localRow++) {
      for (int localCol = 0; localCol < numElasticLocalParams_; localCol++) {
        std::ptrdiff_t offset = idxM(localRow, localCol);
        if (offset >= 0) {
          std::atomic_ref<double> hessRef(hess.valuePtr()[offset]);
          hessRef.fetch_add(localH(localRow, localCol));
        }
      }
    }
  };

  tbb::parallel_for(0, nele, localHessFunc);

  if (enableSanityCheck)
    sanityCheckValues(std::span<double>(hess.valuePtr(), hess.nonZeros()), hess.nonZeros(), "plastic-elastic Hessian");
}

void DeformationModelAssembler::compute_d2E_dudp(
  std::span<const double> absolutePositions, MaterialStateView state,
  EigenSupport::SpMatD &mixedHessian) const
{
  validateMaterialState(state);
  if (numPlasticParams_ == 0)
    return;
  assemble_d2E_dudq(
    absolutePositions, state, numPlasticParams_, numPlasticLocalParams_,
    *plasticField_,
    element_d2E_dudp_InverseIndices,
    &DeformationModelEvaluator::compute_d2E_dudp,
    mixedHessian, "d2E/dudp");
}

void DeformationModelAssembler::compute_d2E_dude(
  std::span<const double> absolutePositions, MaterialStateView state,
  EigenSupport::SpMatD &mixedHessian) const
{
  validateMaterialState(state);
  if (numElasticParams_ == 0)
    return;
  assemble_d2E_dudq(
    absolutePositions, state, numElasticParams_, numElasticLocalParams_,
    *elasticField_,
    element_d2E_dude_InverseIndices,
    &DeformationModelEvaluator::compute_d2E_dude,
    mixedHessian, "d2E/dude");
}

void DeformationModelAssembler::computePlasticMaterialVJP(
  std::span<const double> absolutePositions,
  std::span<const double> adjoint,
  MaterialStateView state,
  std::span<double> output) const
{
  validateMaterialState(state);
  assembleMaterialVJP(
    absolutePositions, adjoint, state,
    numPlasticParams_, numPlasticLocalParams_, *plasticField_,
    &DeformationModelEvaluator::compute_d2E_dudp,
    output, "plastic material VJP");
}

void DeformationModelAssembler::computeElasticMaterialVJP(
  std::span<const double> absolutePositions,
  std::span<const double> adjoint,
  MaterialStateView state,
  std::span<double> output) const
{
  validateMaterialState(state);
  assembleMaterialVJP(
    absolutePositions, adjoint, state,
    numElasticParams_, numElasticLocalParams_, *elasticField_,
    &DeformationModelEvaluator::compute_d2E_dude,
    output, "elastic material VJP");
}

void DeformationModelAssembler::computeVonMisesStresses(
  std::span<const double> x, MaterialStateView state, std::span<double> elementStresses) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  if (nele == 0)
    return;
  if (elementStresses.size() < static_cast<std::size_t>(nele))
    throw std::invalid_argument(
      "Von Mises stress output must not be null.");
  std::fill(elementStresses.begin(), elementStresses.begin() + nele, 0.0);

  auto localStressFunc = [this, x, &state, elementStresses](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elementWorkspaces_[ele];
    DeformationModelEvaluator &evaluator = gatherAndPrepare(ele, x, state, scratch);

    std::fill(scratch.materialLocationValues.begin(), scratch.materialLocationValues.end(), 0.0);
    int nPt = 0;
    try {
      nPt = evaluator.computeVonMisesStress(std::span<double>(scratch.materialLocationValues.data(), scratch.materialLocationValues.size()),
        static_cast<int>(scratch.materialLocationValues.size()));
    }
    catch (const UnsupportedDeformationDiagnosticError &e) {
      throw UnsupportedDeformationDiagnosticError(
        "Element " + std::to_string(ele) + ": " + e.what());
    }

    if (nPt <= 0 ||
      nPt > static_cast<int>(scratch.materialLocationValues.size())) {
      throw std::runtime_error(
        "Element " + std::to_string(ele) +
        " returned an invalid von Mises stress sample count: " +
        std::to_string(nPt) + ".");
    }
    elementStresses[ele] = *std::max_element(
      scratch.materialLocationValues.begin(),
      scratch.materialLocationValues.begin() + nPt);
  };

  tbb::parallel_for(0, nele, localStressFunc);
}

void DeformationModelAssembler::computeMaxStrains(
  std::span<const double> x, MaterialStateView state, std::span<double> elementStrain) const
{
  validatePositionSpan(x, "position vector");
  validateMaterialState(state);
  if (nele == 0)
    return;
  if (elementStrain.size() < static_cast<std::size_t>(nele))
    throw std::invalid_argument(
      "Maximum strain output must not be null.");
  std::fill(elementStrain.begin(), elementStrain.begin() + nele, 0.0);

  auto localStrainFunc = [this, x, &state, elementStrain](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elementWorkspaces_[ele];
    DeformationModelEvaluator &evaluator = gatherAndPrepare(ele, x, state, scratch);

    std::fill(scratch.materialLocationValues.begin(), scratch.materialLocationValues.end(), 0.0);
    int nPt = 0;
    try {
      nPt = evaluator.computeMaxStrain(std::span<double>(scratch.materialLocationValues.data(), scratch.materialLocationValues.size()),
        static_cast<int>(scratch.materialLocationValues.size()));
    }
    catch (const UnsupportedDeformationDiagnosticError &e) {
      throw UnsupportedDeformationDiagnosticError(
        "Element " + std::to_string(ele) + ": " + e.what());
    }

    if (nPt <= 0 ||
      nPt > static_cast<int>(scratch.materialLocationValues.size())) {
      throw std::runtime_error(
        "Element " + std::to_string(ele) +
        " returned an invalid maximum strain sample count: " +
        std::to_string(nPt) + ".");
    }
    elementStrain[ele] = *std::max_element(
      scratch.materialLocationValues.begin(),
      scratch.materialLocationValues.begin() + nPt);
  };

  tbb::parallel_for(0, nele, localStrainFunc);
}

// ── Private helpers ──────────────────────────────────────────────────────────

void DeformationModelAssembler::buildMixedSparsityTemplate(
  int numLocalParams,
  int numGlobalParams,
  const std::function<int(int, int)> &paramGlobalCol,
  EigenSupport::SpMatD &tmpl,
  std::vector<DynamicIndexMatrix> &inverseIndices,
  std::vector<ES::TripletD> &entries)
{
  entries.clear();
  for (int ele = 0; ele < nele; ele++) {
    std::vector<DofGroup> groups;
    dofLayout->getDofGroups(ele, groups);
    for (const DofGroup &group : groups) {
      for (int localOffset = 0; localOffset < group.size; localOffset++) {
        const int globalRow = group.globalDof(localOffset);
        for (int pp = 0; pp < numLocalParams; pp++)
          entries.emplace_back(globalRow, paramGlobalCol(ele, pp), 1.0);
      }
    }
  }
  tmpl.resize(numDOFs, numGlobalParams);
  tmpl.setFromTriplets(entries.begin(), entries.end());

  inverseIndices.resize(nele);
  for (int ele = 0; ele < nele; ele++) {
    DynamicIndexMatrix idxM(localDOFs, numLocalParams);
    idxM.setConstant(-1);

    std::vector<DofGroup> groups;
    dofLayout->getDofGroups(ele, groups);
    for (const DofGroup &group : groups) {
      for (int localOffset = 0; localOffset < group.size; localOffset++) {
        const int localRow = group.localStart + localOffset;
        const int globalRow = group.globalDof(localOffset);
        for (int pp = 0; pp < numLocalParams; pp++) {
          int globalCol = paramGlobalCol(ele, pp);
          idxM(localRow, pp) = ES::findEntryOffset(tmpl, globalRow, globalCol);
        }
      }
    }
    inverseIndices[ele] = idxM;
  }
}

void DeformationModelAssembler::assemble_d2E_dudq(
  std::span<const double> absolutePositions,
  MaterialStateView state,
  int numMaterialParams,
  int numLocalParams,
  const OptimizableParameterField &paramBlock,
  const std::vector<DynamicIndexMatrix> &inverseIndices,
  void (DeformationModelEvaluator::*computeLocal)(
    EigenSupport::RefMatXd, int) const,
  EigenSupport::SpMatD &mixedHessian,
  const char *label) const
{
  validatePositionSpan(absolutePositions, "position vector");
  memset(
    mixedHessian.valuePtr(), 0,
    sizeof(double) * mixedHessian.nonZeros());

  if (numMaterialParams == 0 || numLocalParams == 0)
    return;

  auto localFunc = [this, absolutePositions, &state, &mixedHessian,
                     numMaterialParams, numLocalParams, &paramBlock,
                     &inverseIndices, computeLocal](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elementWorkspaces_[ele];
    DeformationModelEvaluator &evaluator = gatherAndPrepare(
      ele, absolutePositions, state, scratch);

    const ES::Mp<ES::MXd> rawK(scratch.localMatrixData.data(), localDOFs, numMaterialParams);
    auto dParamDLocal = scratch.paramDerivativeData.block(
      0, 0, numMaterialParams, numLocalParams);
    auto localK = scratch.localMixedMatrix.block(0, 0, localDOFs, numLocalParams);
    localK.setZero();
    for (int q = 0; q < femModels[ele].get().getNumMaterialLocations(); q++) {
      (evaluator.*computeLocal)(
        ES::Mp<ES::MXd>(scratch.localMatrixData.data(), localDOFs, numMaterialParams), q);
      fillLocalParamDerivative(
        paramBlock, state, ele, q,
        std::span<double>(scratch.localParamValues.data(), scratch.localParamValues.size()),
        dParamDLocal);
      localK.noalias() += rawK * dParamDLocal;
    }
    localK *= elementWeights[ele];

    const auto &idxM = inverseIndices[ele];
    for (int localRow = 0; localRow < localDOFs; localRow++) {
      for (int va = 0; va < numLocalParams; va++) {
        std::ptrdiff_t offset = idxM(localRow, va);
        if (offset >= 0) {
          std::atomic_ref<double> hessRef(
            mixedHessian.valuePtr()[offset]);
          hessRef.fetch_add(localK(localRow, va));
        }
      }
    }
  };

  for (int ele = 0; ele < nele; ele++) {
    localFunc(ele);
  }

  if (enableSanityCheck)
    sanityCheckValues(
      std::span<double>(mixedHessian.valuePtr(), mixedHessian.nonZeros()),
      mixedHessian.nonZeros(), label);
}

void DeformationModelAssembler::assembleMaterialVJP(
  std::span<const double> absolutePositions,
  std::span<const double> adjoint,
  MaterialStateView state,
  int numMaterialParams,
  int numLocalParams,
  const OptimizableParameterField &paramBlock,
  void (DeformationModelEvaluator::*computeLocal)(
    EigenSupport::RefMatXd, int) const,
  std::span<double> output,
  const char *label) const
{
  validatePositionSpan(absolutePositions, "position vector");
  validatePositionSpan(adjoint, "material VJP adjoint");
  const int numGlobalParams = paramBlock.layout().numGlobalParameters();
  if (output.size() != static_cast<std::size_t>(numGlobalParams))
    throw std::invalid_argument(
      "DeformationModelAssembler material VJP output has unexpected size.");
  std::fill(output.begin(), output.end(), 0.0);

  if (numMaterialParams == 0 || numLocalParams == 0)
    return;

  auto localFunc = [this, absolutePositions, adjoint, &state, output,
                     numMaterialParams, numLocalParams, &paramBlock,
                     computeLocal](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = elementWorkspaces_[ele];
    DeformationModelEvaluator &evaluator = gatherAndPrepare(
      ele, absolutePositions, state, scratch);
    std::fill(
      scratch.localDirection.data(),
      scratch.localDirection.data() + localDOFs, 0.0);
    for (const DofGroup &group : scratch.groups)
      for (int i = 0; i < group.size; ++i)
        scratch.localDirection[group.localStart + i] =
          adjoint[group.globalDof(i)];

    const ES::Mp<ES::MXd> rawK(
      scratch.localMatrixData.data(), localDOFs, numMaterialParams);
    auto dParamDLocal = scratch.paramDerivativeData.block(
      0, 0, numMaterialParams, numLocalParams);
    auto localK = scratch.localMixedMatrix.block(
      0, 0, localDOFs, numLocalParams);
    localK.setZero();
    for (int q = 0; q < femModels[ele].get().getNumMaterialLocations(); ++q) {
      (evaluator.*computeLocal)(
        ES::Mp<ES::MXd>(scratch.localMatrixData.data(),
          localDOFs, numMaterialParams), q);
      fillLocalParamDerivative(
        paramBlock, state, ele, q,
        std::span<double>(scratch.localParamValues.data(),
          scratch.localParamValues.size()),
        dParamDLocal);
      localK.noalias() += rawK * dParamDLocal;
    }
    localK *= elementWeights[ele];

    auto localVJP = scratch.localParamGradient.head(numLocalParams);
    localVJP.noalias() = localK.transpose() * scratch.localDirection;
    for (int localParam = 0; localParam < numLocalParams; ++localParam) {
      const int globalParam =
        paramBlock.layout().globalParameter(ele, localParam);
      std::atomic_ref<double> outputRef(output[globalParam]);
      outputRef.fetch_add(localVJP[localParam]);
    }
  };

  tbb::parallel_for(0, nele, localFunc);

  if (enableSanityCheck)
    sanityCheckValues(output, output.size(), label);
}
