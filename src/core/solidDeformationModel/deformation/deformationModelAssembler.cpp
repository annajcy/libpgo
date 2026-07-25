/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "deformation/deformationModelAssembler.h"
#include "deformation/deformationModelAssemblerCacheData.h"
#include "deformation/materialMaxStepPolynomialUtils.h"
#include "deformation/deformationModelManager.h"
#include "simulation/simulationMesh.h"
#include "deformation/deformationModel.h"
#include "material/elastic/elasticModel.h"
#include "material/plastic/plasticModel.h"
#include "material/core/materialParameters.h"

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

void sanityCheckValues(double *values, Eigen::Index count, const char *label)
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
  const MaterialParameterField &block,
  const MaterialParameterEvaluationView &state,
  int ele,
  int quadratureId,
  double *localDofValues,
  double *derivOut)
{
  const auto &layout = block.dofLayout();
  const auto &mapping = block.channelMapping();
  const int numChannels = mapping.numChannels();
  const int numLocalDofs = layout.numLocalDofs();
  if (numLocalDofs > 0) {
    layout.gather(
      ele, state.values(block),
      std::span<double>(localDofValues, numLocalDofs));
  }
  if (numChannels > 0 && numLocalDofs > 0) {
    mapping.evaluateJacobian(
      ele, quadratureId,
      std::span<const double>(localDofValues, numLocalDofs),
      derivOut);
  }
}

void fillElementParamValues(
  const MaterialParameterField &block,
  const MaterialParameterEvaluationView &state,
  int ele,
  int numMaterialLocations,
  double *localDofValues,
  double *values)
{
  const auto &layout = block.dofLayout();
  const auto &mapping = block.channelMapping();
  const int numChannels = mapping.numChannels();
  const int numLocalDofs = layout.numLocalDofs();
  if (numChannels == 0 || numMaterialLocations == 0) {
    return;
  }

  layout.gather(
    ele, state.values(block),
    std::span<double>(localDofValues, numLocalDofs));
  for (int q = 0; q < numMaterialLocations; q++) {
    mapping.evaluate(
      ele, q,
      std::span<const double>(localDofValues, numLocalDofs),
      std::span<double>(
        values + static_cast<std::ptrdiff_t>(q) * numChannels,
        numChannels));
  }
}
}  // namespace

DeformationModelAssembler::DeformationModelAssembler(
  std::shared_ptr<DeformationModelManager> dm,
  const Formulation &formulation,
  std::shared_ptr<const MaterialParameterSpace> materialParameterSpace,
  const double *elementWeights_):
  deformationModelManager(std::move(dm)),
  dofLayout(formulation.createDofLayout(*deformationModelManager->getMesh())),
  restDofs_(formulation.buildGlobalRestDofs(*deformationModelManager->getMesh())),
  materialParameterSpace_(std::move(materialParameterSpace))
{
  if (!materialParameterSpace_)
    throw std::invalid_argument("DeformationModelAssembler requires a material parameter space.");
  nele = deformationModelManager->getMesh()->getNumElements();
  neleVtx = deformationModelManager->getMesh()->getNumElementVertices();
  localDOFs = dofLayout->numLocalDofs(0);
  numDOFs = dofLayout->numGlobalDofs();

  numElasticParams_ = deformationModelManager->getDeformationModel(0)->getNumElasticParameters();
  numPlasticParams_ = deformationModelManager->getDeformationModel(0)->getNumPlasticParameters();
  const auto &elasticBlock = materialParameterSpace_->elastic();
  const auto &plasticBlock = materialParameterSpace_->plastic();
  if (elasticBlock.channelMapping().numChannels() != numElasticParams_)
    throw std::invalid_argument("DeformationModelAssembler elastic channel count does not match the material model.");
  if (plasticBlock.channelMapping().numChannels() != numPlasticParams_)
    throw std::invalid_argument("DeformationModelAssembler plastic channel count does not match the material model.");
  if (elasticBlock.dofLayout().numElements() != nele ||
    plasticBlock.dofLayout().numElements() != nele)
    throw std::invalid_argument("DeformationModelAssembler parameter layout element count does not match the mesh.");

  const auto *elasticParamLayout = &elasticBlock.dofLayout();
  const auto *plasticParamLayout = &plasticBlock.dofLayout();
  numElasticLocalParams_ = elasticParamLayout->numLocalDofs();
  numPlasticLocalParams_ = plasticParamLayout->numLocalDofs();

  if (elementWeights_) {
    elementWeights.assign(elementWeights_, elementWeights_ + nele);
  }
  else {
    elementWeights.assign(nele, 1);
  }

  int maxMaterialLocations = 0;
  for (int i = 0; i < nele; i++) {
    femModels.push_back(deformationModelManager->getDeformationModel(i));
    maxMaterialLocations = std::max(maxMaterialLocations, femModels.back()->getNumMaterialLocations());
  }

  const int maxMaterialParams = std::max(numElasticParams_, numPlasticParams_);
  const int maxLocalParams = std::max(numElasticLocalParams_, numPlasticLocalParams_);
  const auto mappingHessianEntries = [](const MaterialParameterField &block) {
    if (block.channelMapping().isAffine())
      return std::size_t(0);
    const std::size_t channels = static_cast<std::size_t>(
      block.channelMapping().numChannels());
    const std::size_t localParams = static_cast<std::size_t>(
      block.dofLayout().numLocalDofs());
    return channels * localParams * localParams;
  };
  const std::size_t maxMappingHessianEntries = std::max(
    mappingHessianEntries(elasticBlock), mappingHessianEntries(plasticBlock));
  data = std::make_unique<DeformationModelAssemblerCacheData>(
    localDOFs, maxMaterialLocations, maxMaterialParams, maxLocalParams,
    maxMappingHessianEntries, femModels);
  for (int ele = 0; ele < nele; ele++)
    dofLayout->getDofGroups(ele, data->elementScratch(ele).groups);
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
  const int numElasticGlobalParams = elasticParamLayout ? elasticParamLayout->numGlobalDofs() : 0;
  if (numElasticParams_ > 0 && numElasticLocalParams_ > 0 && elasticParamLayout) {
    buildMixedSparsityTemplate(
      numElasticLocalParams_, numElasticGlobalParams,
      [elasticParamLayout](int ele, int ep) { return elasticParamLayout->globalDof(ele, ep); },
      d2E_dudeTemplate, element_d2E_dude_InverseIndices, entries);
  }
  else {
    d2E_dudeTemplate.resize(numDOFs, 0);
  }

  // Displacement-plastic Hessian template.
  const int numPlasticGlobalParams = plasticParamLayout ? plasticParamLayout->numGlobalDofs() : 0;
  if (numPlasticParams_ > 0 && numPlasticLocalParams_ > 0 && plasticParamLayout) {
    buildMixedSparsityTemplate(
      numPlasticLocalParams_, numPlasticGlobalParams,
      [plasticParamLayout](int ele, int pp) { return plasticParamLayout->globalDof(ele, pp); },
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
        const int globalRow = plasticParamLayout->globalDof(ele, pi);
        for (int pj = 0; pj < numPlasticLocalParams_; pj++) {
          const int globalCol = plasticParamLayout->globalDof(ele, pj);
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
        const int globalRow = plasticParamLayout->globalDof(ele, pi);
        for (int pj = 0; pj < numPlasticLocalParams_; pj++) {
          const int globalCol = plasticParamLayout->globalDof(ele, pj);
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
        const int globalRow = elasticParamLayout->globalDof(ele, pi);
        for (int pj = 0; pj < numElasticLocalParams_; pj++) {
          const int globalCol = elasticParamLayout->globalDof(ele, pj);
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
        const int globalRow = elasticParamLayout->globalDof(ele, pi);
        for (int pj = 0; pj < numElasticLocalParams_; pj++) {
          const int globalCol = elasticParamLayout->globalDof(ele, pj);
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
        const int globalRow = plasticParamLayout->globalDof(ele, pi);
        for (int ej = 0; ej < numElasticLocalParams_; ej++) {
          const int globalCol = elasticParamLayout->globalDof(ele, ej);
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
        const int globalRow = plasticParamLayout->globalDof(ele, pi);
        for (int ej = 0; ej < numElasticLocalParams_; ej++) {
          const int globalCol = elasticParamLayout->globalDof(ele, ej);
          idxM(pi, ej) = ES::findEntryOffset(
            d2E_dpdeTemplate, globalRow, globalCol);
        }
      }

      element_d2E_dpde_InverseIndices[ele] = idxM;
    }
  }
}

DeformationModelAssembler::~DeformationModelAssembler() = default;

void DeformationModelAssembler::validateMaterialParameterSnapshot(
  const MaterialParameterEvaluationView &state) const
{
  if (state.empty())
    throw std::invalid_argument("DeformationModelAssembler requires a non-empty material state.");
  if (&state.space() != materialParameterSpace_.get())
    throw std::invalid_argument("MaterialParameterEvaluationView belongs to a different material parameter space.");
  if (state.elasticValues().size() != static_cast<std::size_t>(getNumElasticGlobalParams()))
    throw std::invalid_argument("MaterialParameterEvaluationView elastic value count does not match the assembler.");
  if (state.plasticValues().size() != static_cast<std::size_t>(getNumPlasticGlobalParams()))
    throw std::invalid_argument("MaterialParameterEvaluationView plastic value count does not match the assembler.");
}

DeformationModelAssembler::PreparedElement DeformationModelAssembler::gatherAndPrepare(
  int ele, const double *x, const MaterialParameterEvaluationView &state,
  DeformationModelAssemblerCacheData::ElementScratch &scratch) const
{
  std::fill(scratch.localPosition.data(), scratch.localPosition.data() + localDOFs, 0.0);
  for (const DofGroup &group : scratch.groups) {
    for (int i = 0; i < group.size; i++)
      scratch.localPosition[group.localStart + i] = x[group.globalDof(i)];
  }
  const DeformationModel *fem = femModels[ele];
  DeformationModel::CacheData *cache = scratch.cacheData();
  cache->markUnprepared();
  const int numMaterialLocations = fem->getNumMaterialLocations();
  fillElementParamValues(
    materialParameterSpace_->elastic(), state, ele,
    numMaterialLocations, scratch.localParamValues.data(),
    scratch.elasticParamValues.data());
  fillElementParamValues(
    materialParameterSpace_->plastic(), state, ele,
    numMaterialLocations, scratch.localParamValues.data(),
    scratch.plasticParamValues.data());

  fem->prepareData(scratch.localPosition.data(),
    numElasticParams_ > 0 ? scratch.elasticParamValues.data() : nullptr,
    numPlasticParams_ > 0 ? scratch.plasticParamValues.data() : nullptr,
    cache);
  return { fem, cache };
}

double DeformationModelAssembler::computeEnergy(
  const double *x, MaterialParameterEvaluationView state) const
{
  validateMaterialParameterSnapshot(state);

  auto localEnergyFunc = [this, x, &state](int ele) {
    auto &scratch = data->elementScratch(ele);
    scratch.energy = 0.0;
    if (elementWeights[ele] == 0)
      return;

    PreparedElement prepared = gatherAndPrepare(ele, x, state, scratch);
    double energy = prepared.model->computeEnergy(prepared.cache);

    scratch.energy = energy * elementWeights[ele];
  };

  tbb::parallel_for(0, nele, localEnergyFunc);

  double energyAll = 0;
  for (int ele = 0; ele < nele; ele++)
    energyAll += data->elementScratch(ele).energy;

  return energyAll;
}

DeformationModelAssembler::MaterialMaxStepObservation DeformationModelAssembler::computeMaxStepObservation(const double *x, const double *dx) const
{
  MaterialMaxStepObservation observation;
  const SimulationMeshType meshType = deformationModelManager->getMesh()->getElementType();

  for (int ele = 0; ele < nele; ele++) {
    if (elementWeights[ele] == 0) {
      continue;
    }

    auto &scratch = data->elementScratch(ele);
    std::fill(scratch.localPosition.data(), scratch.localPosition.data() + localDOFs, 0.0);
    std::fill(scratch.localDirection.data(), scratch.localDirection.data() + localDOFs, 0.0);
    for (const DofGroup &group : scratch.groups) {
      for (int i = 0; i < group.size; i++) {
        scratch.localPosition[group.localStart + i] = x[group.globalDof(i)];
        scratch.localDirection[group.localStart + i] = dx[group.globalDof(i)];
      }
    }

    const DeformationModel::LocalMaxStepResult localResult =
      femModels[ele]->computeLocalMaxStepSize(scratch.localPosition.data(), scratch.localDirection.data());
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

double DeformationModelAssembler::computeMaxStepSize(const double *x, const double *dx) const
{
  return computeMaxStepObservation(x, dx).alpha;
}

void DeformationModelAssembler::computeGradient(
  const double *x, MaterialParameterEvaluationView state, double *grad) const
{
  validateMaterialParameterSnapshot(state);
  memset(grad, 0, sizeof(double) * numDOFs);
  auto localGradFunc = [this, x, &state, grad](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->elementScratch(ele);
    PreparedElement prepared = gatherAndPrepare(ele, x, state, scratch);

    prepared.model->compute_dE_dx(prepared.cache, scratch.localGradient.data());
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
    sanityCheckValues(grad, numDOFs, "gradient");
}

void DeformationModelAssembler::computeHessian(
  const double *x, MaterialParameterEvaluationView state, EigenSupport::SpMatD &hess) const
{
  validateMaterialParameterSnapshot(state);
  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  auto localHessFunc = [this, x, &state, &hess](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->elementScratch(ele);
    PreparedElement prepared = gatherAndPrepare(ele, x, state, scratch);

    prepared.model->compute_d2E_dx2(prepared.cache, scratch.localMatrixData.data());

    ES::Mp<ES::MXd> localK(scratch.localMatrixData.data(), localDOFs, localDOFs);
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
    sanityCheckValues(hess.valuePtr(), hess.nonZeros(), "Hessian");
}

int DeformationModelAssembler::getNumElasticGlobalParams() const
{
  return materialParameterSpace_->elastic().dofLayout().numGlobalDofs();
}

int DeformationModelAssembler::getNumPlasticGlobalParams() const
{
  return materialParameterSpace_->plastic().dofLayout().numGlobalDofs();
}

void DeformationModelAssembler::compute_dE_dp(
  const double *x, MaterialParameterEvaluationView state, double *grad) const
{
  validateMaterialParameterSnapshot(state);
  const int numPlasticGlobalParams = getNumPlasticGlobalParams();
  std::fill(grad, grad + numPlasticGlobalParams, 0.0);

  if (numPlasticParams_ == 0 || numPlasticLocalParams_ == 0 || numPlasticGlobalParams == 0)
    return;

  const auto &plasticBlock = materialParameterSpace_->plastic();
  const auto *plasticParamLayout = &plasticBlock.dofLayout();

  auto localGradFunc = [this, x, &state, grad, plasticParamLayout, &plasticBlock](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->elementScratch(ele);
    PreparedElement prepared = gatherAndPrepare(ele, x, state, scratch);

    Eigen::Map<ES::VXd> rawGrad(
      scratch.rawParamGradient.data(), numPlasticParams_);
    const Eigen::Map<const ES::MXd> dParamDLocal(
      scratch.paramDerivativeData.data(), numPlasticParams_, numPlasticLocalParams_);
    auto localGrad =
      scratch.localParamGradient.head(numPlasticLocalParams_);
    localGrad.setZero();
    for (int q = 0; q < prepared.model->getNumMaterialLocations(); q++) {
      rawGrad.setZero();
      prepared.model->compute_dE_dp(prepared.cache, rawGrad.data(), q);
      fillLocalParamDerivative(
        plasticBlock, state, ele, q,
        scratch.localParamValues.data(),
        scratch.paramDerivativeData.data());
      localGrad.noalias() += dParamDLocal.transpose() * rawGrad;
    }
    localGrad *= elementWeights[ele];

    for (int pi = 0; pi < numPlasticLocalParams_; pi++) {
      const int globalRow = plasticParamLayout->globalDof(ele, pi);
      std::atomic_ref<double> gradRef(grad[globalRow]);
      gradRef.fetch_add(scratch.localParamGradient[pi]);
    }
  };

  tbb::parallel_for(0, nele, localGradFunc);

  if (enableSanityCheck)
    sanityCheckValues(grad, numPlasticGlobalParams, "plastic gradient");
}

void DeformationModelAssembler::compute_d2E_dp2(
  const double *x, MaterialParameterEvaluationView state, EigenSupport::SpMatD &hess) const
{
  validateMaterialParameterSnapshot(state);
  if (hess.rows() != d2E_dp2Template.rows() ||
    hess.cols() != d2E_dp2Template.cols() ||
    hess.nonZeros() != d2E_dp2Template.nonZeros()) {
    hess = d2E_dp2Template;
  }

  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  const int numPlasticGlobalParams = getNumPlasticGlobalParams();
  if (numPlasticParams_ == 0 || numPlasticLocalParams_ == 0 || numPlasticGlobalParams == 0)
    return;

  const auto &plasticBlock = materialParameterSpace_->plastic();
  auto localHessFunc = [this, x, &state, &hess, &plasticBlock](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->elementScratch(ele);
    PreparedElement prepared = gatherAndPrepare(ele, x, state, scratch);

    const ES::Mp<ES::MXd> rawH(scratch.localMatrixData.data(), numPlasticParams_, numPlasticParams_);
    const Eigen::Map<const ES::MXd> dParamDLocal(
      scratch.paramDerivativeData.data(), numPlasticParams_, numPlasticLocalParams_);
    auto paramWork = scratch.paramWorkMatrix.block(0, 0, numPlasticParams_, numPlasticLocalParams_);
    auto localH = scratch.localParamHessian.block(0, 0, numPlasticLocalParams_, numPlasticLocalParams_);
    localH.setZero();
    for (int q = 0; q < prepared.model->getNumMaterialLocations(); q++) {
      prepared.model->compute_d2E_dp2(
        prepared.cache, scratch.localMatrixData.data(), q);
      fillLocalParamDerivative(
        plasticBlock, state, ele, q,
        scratch.localParamValues.data(),
        scratch.paramDerivativeData.data());
      paramWork.noalias() = rawH * dParamDLocal;
      localH.noalias() += dParamDLocal.transpose() * paramWork;
      if (!plasticBlock.channelMapping().isAffine()) {
        Eigen::Map<ES::VXd> rawGrad(
          scratch.rawParamGradient.data(), numPlasticParams_);
        rawGrad.setZero();
        prepared.model->compute_dE_dp(
          prepared.cache, rawGrad.data(), q);
        plasticBlock.channelMapping().evaluateHessians(
          ele, q,
          std::span<const double>(
            scratch.localParamValues.data(), numPlasticLocalParams_),
          scratch.paramMappingHessianData.data());
        const std::size_t channelStride =
          static_cast<std::size_t>(numPlasticLocalParams_) *
          numPlasticLocalParams_;
        for (int channel = 0; channel < numPlasticParams_; channel++) {
          const Eigen::Map<const ES::MXd> mappingHessian(
            scratch.paramMappingHessianData.data() +
              static_cast<std::size_t>(channel) * channelStride,
            numPlasticLocalParams_, numPlasticLocalParams_);
          localH.noalias() += rawGrad[channel] * mappingHessian;
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
    sanityCheckValues(hess.valuePtr(), hess.nonZeros(), "plastic Hessian");
}

void DeformationModelAssembler::compute_dE_de(
  const double *x, MaterialParameterEvaluationView state, double *grad) const
{
  validateMaterialParameterSnapshot(state);
  const int numElasticGlobalParams = getNumElasticGlobalParams();
  std::fill(grad, grad + numElasticGlobalParams, 0.0);

  if (numElasticParams_ == 0 || numElasticLocalParams_ == 0 || numElasticGlobalParams == 0)
    return;

  const auto &elasticBlock = materialParameterSpace_->elastic();
  const auto *elasticParamLayout = &elasticBlock.dofLayout();

  auto localGradFunc = [this, x, &state, grad, elasticParamLayout, &elasticBlock](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->elementScratch(ele);
    PreparedElement prepared = gatherAndPrepare(ele, x, state, scratch);

    Eigen::Map<ES::VXd> rawGrad(
      scratch.rawParamGradient.data(), numElasticParams_);
    const Eigen::Map<const ES::MXd> dParamDLocal(
      scratch.paramDerivativeData.data(), numElasticParams_, numElasticLocalParams_);
    auto localGrad =
      scratch.localParamGradient.head(numElasticLocalParams_);
    localGrad.setZero();
    for (int q = 0; q < prepared.model->getNumMaterialLocations(); q++) {
      rawGrad.setZero();
      prepared.model->compute_dE_de(prepared.cache, rawGrad.data(), q);
      fillLocalParamDerivative(
        elasticBlock, state, ele, q,
        scratch.localParamValues.data(),
        scratch.paramDerivativeData.data());
      localGrad.noalias() += dParamDLocal.transpose() * rawGrad;
    }
    localGrad *= elementWeights[ele];

    for (int pi = 0; pi < numElasticLocalParams_; pi++) {
      const int globalRow = elasticParamLayout->globalDof(ele, pi);
      std::atomic_ref<double> gradRef(grad[globalRow]);
      gradRef.fetch_add(scratch.localParamGradient[pi]);
    }
  };

  tbb::parallel_for(0, nele, localGradFunc);

  if (enableSanityCheck)
    sanityCheckValues(grad, numElasticGlobalParams, "elastic gradient");
}

void DeformationModelAssembler::compute_d2E_de2(
  const double *x, MaterialParameterEvaluationView state, EigenSupport::SpMatD &hess) const
{
  validateMaterialParameterSnapshot(state);
  if (hess.rows() != d2E_de2Template.rows() ||
    hess.cols() != d2E_de2Template.cols() ||
    hess.nonZeros() != d2E_de2Template.nonZeros()) {
    hess = d2E_de2Template;
  }

  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  const int numElasticGlobalParams = getNumElasticGlobalParams();
  if (numElasticParams_ == 0 || numElasticLocalParams_ == 0 || numElasticGlobalParams == 0)
    return;

  const auto &elasticBlock = materialParameterSpace_->elastic();
  auto localHessFunc = [this, x, &state, &hess, &elasticBlock](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->elementScratch(ele);
    PreparedElement prepared = gatherAndPrepare(ele, x, state, scratch);

    const ES::Mp<ES::MXd> rawH(scratch.localMatrixData.data(), numElasticParams_, numElasticParams_);
    const Eigen::Map<const ES::MXd> dParamDLocal(
      scratch.paramDerivativeData.data(), numElasticParams_, numElasticLocalParams_);
    auto paramWork = scratch.paramWorkMatrix.block(0, 0, numElasticParams_, numElasticLocalParams_);
    auto localH = scratch.localParamHessian.block(0, 0, numElasticLocalParams_, numElasticLocalParams_);
    localH.setZero();
    for (int q = 0; q < prepared.model->getNumMaterialLocations(); q++) {
      prepared.model->compute_d2E_de2(
        prepared.cache, scratch.localMatrixData.data(), q);
      fillLocalParamDerivative(
        elasticBlock, state, ele, q,
        scratch.localParamValues.data(),
        scratch.paramDerivativeData.data());
      paramWork.noalias() = rawH * dParamDLocal;
      localH.noalias() += dParamDLocal.transpose() * paramWork;
      if (!elasticBlock.channelMapping().isAffine()) {
        Eigen::Map<ES::VXd> rawGrad(
          scratch.rawParamGradient.data(), numElasticParams_);
        rawGrad.setZero();
        prepared.model->compute_dE_de(
          prepared.cache, rawGrad.data(), q);
        elasticBlock.channelMapping().evaluateHessians(
          ele, q,
          std::span<const double>(
            scratch.localParamValues.data(), numElasticLocalParams_),
          scratch.paramMappingHessianData.data());
        const std::size_t channelStride =
          static_cast<std::size_t>(numElasticLocalParams_) *
          numElasticLocalParams_;
        for (int channel = 0; channel < numElasticParams_; channel++) {
          const Eigen::Map<const ES::MXd> mappingHessian(
            scratch.paramMappingHessianData.data() +
              static_cast<std::size_t>(channel) * channelStride,
            numElasticLocalParams_, numElasticLocalParams_);
          localH.noalias() += rawGrad[channel] * mappingHessian;
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
    sanityCheckValues(hess.valuePtr(), hess.nonZeros(), "elastic Hessian");
}

void DeformationModelAssembler::compute_d2E_dpde(
  const double *x, MaterialParameterEvaluationView state, EigenSupport::SpMatD &hess) const
{
  validateMaterialParameterSnapshot(state);
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

  const auto &plasticBlock = materialParameterSpace_->plastic();
  const auto &elasticBlock = materialParameterSpace_->elastic();
  auto localHessFunc = [this, x, &state, &hess, &plasticBlock, &elasticBlock](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->elementScratch(ele);
    PreparedElement prepared = gatherAndPrepare(ele, x, state, scratch);

    const ES::Mp<ES::MXd> rawH(scratch.localMatrixData.data(), numPlasticParams_, numElasticParams_);
    const Eigen::Map<const ES::MXd> dPlasticDLocal(
      scratch.paramDerivativeData.data(), numPlasticParams_, numPlasticLocalParams_);
    const Eigen::Map<const ES::MXd> dElasticDLocal(
      scratch.paramDerivativeData2.data(), numElasticParams_, numElasticLocalParams_);

    auto paramWork = scratch.paramWorkMatrix.block(0, 0, numPlasticParams_, numElasticLocalParams_);
    auto localH = scratch.localParamHessian.block(0, 0, numPlasticLocalParams_, numElasticLocalParams_);
    localH.setZero();
    for (int q = 0; q < prepared.model->getNumMaterialLocations(); q++) {
      prepared.model->compute_d2E_dpde(
        prepared.cache, scratch.localMatrixData.data(), q);
      fillLocalParamDerivative(
        plasticBlock, state, ele, q,
        scratch.localParamValues.data(),
        scratch.paramDerivativeData.data());
      fillLocalParamDerivative(
        elasticBlock, state, ele, q,
        scratch.localParamValues.data(),
        scratch.paramDerivativeData2.data());
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
    sanityCheckValues(hess.valuePtr(), hess.nonZeros(), "plastic-elastic Hessian");
}

void DeformationModelAssembler::compute_d2E_dudp(
  const double *absolutePositions, MaterialParameterEvaluationView state,
  EigenSupport::SpMatD &mixedHessian) const
{
  validateMaterialParameterSnapshot(state);
  if (numPlasticParams_ == 0)
    return;
  assemble_d2E_dudq(
    absolutePositions, state, numPlasticParams_, numPlasticLocalParams_,
    materialParameterSpace_->plastic(),
    element_d2E_dudp_InverseIndices,
    &DeformationModel::compute_d2E_dudp,
    mixedHessian, "d2E/dudp");
}

void DeformationModelAssembler::compute_d2E_dude(
  const double *absolutePositions, MaterialParameterEvaluationView state,
  EigenSupport::SpMatD &mixedHessian) const
{
  validateMaterialParameterSnapshot(state);
  if (numElasticParams_ == 0)
    return;
  assemble_d2E_dudq(
    absolutePositions, state, numElasticParams_, numElasticLocalParams_,
    materialParameterSpace_->elastic(),
    element_d2E_dude_InverseIndices,
    &DeformationModel::compute_d2E_dude,
    mixedHessian, "d2E/dude");
}

void DeformationModelAssembler::computeVonMisesStresses(
  const double *x, MaterialParameterEvaluationView state, double *elementStresses) const
{
  validateMaterialParameterSnapshot(state);
  if (nele == 0)
    return;
  if (elementStresses == nullptr)
    throw std::invalid_argument(
      "Von Mises stress output must not be null.");
  std::fill(elementStresses, elementStresses + nele, 0.0);

  auto localStressFunc = [this, x, &state, elementStresses](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->elementScratch(ele);
    PreparedElement prepared = gatherAndPrepare(ele, x, state, scratch);

    std::fill(scratch.materialLocationValues.begin(), scratch.materialLocationValues.end(), 0.0);
    int nPt = 0;
    try {
      nPt = prepared.model->computeVonMisesStress(
        prepared.cache, scratch.materialLocationValues.data(),
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
  const double *x, MaterialParameterEvaluationView state, double *elementStrain) const
{
  validateMaterialParameterSnapshot(state);
  if (nele == 0)
    return;
  if (elementStrain == nullptr)
    throw std::invalid_argument(
      "Maximum strain output must not be null.");
  std::fill(elementStrain, elementStrain + nele, 0.0);

  auto localStrainFunc = [this, x, &state, elementStrain](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->elementScratch(ele);
    PreparedElement prepared = gatherAndPrepare(ele, x, state, scratch);

    std::fill(scratch.materialLocationValues.begin(), scratch.materialLocationValues.end(), 0.0);
    int nPt = 0;
    try {
      nPt = prepared.model->computeMaxStrain(
        prepared.cache, scratch.materialLocationValues.data(),
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
  const double *absolutePositions,
  MaterialParameterEvaluationView state,
  int numMaterialParams,
  int numLocalParams,
  const MaterialParameterField &paramBlock,
  const std::vector<DynamicIndexMatrix> &inverseIndices,
  void (DeformationModel::*computeLocal)(
    const DeformationModel::CacheData *, double *, int) const,
  EigenSupport::SpMatD &mixedHessian,
  const char *label) const
{
  memset(
    mixedHessian.valuePtr(), 0,
    sizeof(double) * mixedHessian.nonZeros());

  if (numMaterialParams == 0 || numLocalParams == 0)
    return;

  auto localFunc = [
    this, absolutePositions, &state, &mixedHessian,
    numMaterialParams, numLocalParams, &paramBlock,
    &inverseIndices, computeLocal](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->elementScratch(ele);
    PreparedElement prepared = gatherAndPrepare(
      ele, absolutePositions, state, scratch);

    const ES::Mp<ES::MXd> rawK(scratch.localMatrixData.data(), localDOFs, numMaterialParams);
    const Eigen::Map<const ES::MXd> dParamDLocal(
      scratch.paramDerivativeData.data(), numMaterialParams, numLocalParams);
    auto localK = scratch.localMixedMatrix.block(0, 0, localDOFs, numLocalParams);
    localK.setZero();
    for (int q = 0; q < prepared.model->getNumMaterialLocations(); q++) {
      (prepared.model->*computeLocal)(
        prepared.cache, scratch.localMatrixData.data(), q);
      fillLocalParamDerivative(
        paramBlock, state, ele, q,
        scratch.localParamValues.data(),
        scratch.paramDerivativeData.data());
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
      mixedHessian.valuePtr(), mixedHessian.nonZeros(), label);
}
