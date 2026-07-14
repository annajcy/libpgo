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
#include "material/fields/parameterField.h"

#include "pgoLogging.h"
#include "EigenSupport.h"
#include "fmtEigen.h"
#include "processMemory.h"
#include "parallel/parallelFor.h"

#include <algorithm>
#include <atomic>
#include <set>
#include <stdexcept>
#include <string>

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
  const OptimizableField &field, int ele, int quadratureId, int numChannels, int numLocalDofs, double *derivOut)
{
  if (numChannels > 0 && numLocalDofs > 0) {
    std::fill(derivOut, derivOut + static_cast<std::ptrdiff_t>(numChannels) * numLocalDofs, 0.0);
    field.computeDerivative(ele, quadratureId, derivOut);
  }
}

void fillElementParamValues(
  const OptimizableField *field, int ele, int numMaterialLocations, int numChannels, double *values)
{
  if (!field || numChannels == 0 || numMaterialLocations == 0) {
    return;
  }

  for (int q = 0; q < numMaterialLocations; q++) {
    field->computeValue(ele, q, values + static_cast<std::ptrdiff_t>(q) * numChannels);
  }
}

ES::VXd snapshot(const OptimizableField &field)
{
  const auto *layout = field.dofLayout();
  const int n = layout ? layout->numGlobalDofs() : 0;
  ES::VXd out(n);
  if (n > 0)
    out = Eigen::Map<const ES::VXd>(field.globalData(), n);
  return out;
}

void setFieldValues(OptimizableField &field, ES::ConstRefVecXd values, const char *name)
{
  const auto *layout = field.dofLayout();
  const int expected = layout ? layout->numGlobalDofs() : 0;
  if (values.size() != expected)
    throw std::invalid_argument(std::string(name) + ": values size does not match field DOF count.");
  field.setGlobalData(values.data());
}

void validateParameterField(const char *name, const OptimizableField *field,
  ParameterDomain expectedDomain, int expectedChannels, int expectedElements)
{
  if (!field)
    throw std::invalid_argument(std::string(name) + " must be non-null.");

  const auto &spec = field->spec();
  if (spec.domain != expectedDomain)
    throw std::invalid_argument(std::string(name) + " has an incompatible domain.");

  if (field->numChannels() != expectedChannels || spec.numChannels != expectedChannels)
    throw std::invalid_argument(std::string(name) + " channel count does not match the material model.");

  const auto *layout = field->dofLayout();
  if (!layout)
    throw std::invalid_argument(std::string(name) + " must provide a DOF layout.");

  if (!layout->matchesParameterShape(expectedChannels, expectedElements))
    throw std::invalid_argument(std::string(name) + " global DOF count does not match the mesh.");
}
}  // namespace

namespace pgo
{
namespace SolidDeformationModel
{
DeformationModelAssemblerCacheData::ThreadScratch::ThreadScratch(
  int localDofs, int maxMaterialLocations, int maxMaterialParams, int maxLocalParams)
{
  localPosition.resize(localDofs);
  localDirection.resize(localDofs);
  localGradient.resize(localDofs);

  elasticParamValues.resize(maxMaterialLocations * maxMaterialParams);
  plasticParamValues.resize(maxMaterialLocations * maxMaterialParams);
  rawParamGradient.resize(maxMaterialParams);
  localParamGradient.resize(maxLocalParams);

  localParamHessian.resize(maxLocalParams, maxLocalParams);
  paramWorkMatrix.resize(maxMaterialParams, maxLocalParams);
  localMixedMatrix.resize(localDofs, maxLocalParams);
  paramDerivativeData.resize(static_cast<size_t>(maxMaterialParams) * maxLocalParams);
  paramDerivativeData2.resize(static_cast<size_t>(maxMaterialParams) * maxLocalParams);
  localMatrixData.resize(static_cast<size_t>(std::max({
    localDofs * localDofs,
    localDofs * maxMaterialParams,
    maxMaterialParams * maxMaterialParams
  })));
  materialLocationValues.resize(std::max(16, maxMaterialLocations));
  globalDofIndices.resize(localDofs);
}

DeformationModel::CacheData *
DeformationModelAssemblerCacheData::ThreadScratch::cacheFor(const DeformationModel &model)
{
  for (auto &cacheData : reusableCacheData) {
    if (model.isCacheDataCompatible(*cacheData)) {
      cacheData->markUnprepared();
      return cacheData.get();
    }
  }

  reusableCacheData.push_back(model.allocateCacheData());
  return reusableCacheData.back().get();
}

DeformationModelAssemblerCacheData::DeformationModelAssemblerCacheData(
  int localDofs, int maxMaterialLocations, int maxMaterialParams, int maxLocalParams)
{
  threadScratch_ = std::make_unique<tbb::enumerable_thread_specific<ThreadScratch>>(
    [=] { return ThreadScratch(localDofs, maxMaterialLocations, maxMaterialParams, maxLocalParams); });
}

DeformationModelAssemblerCacheData::ThreadScratch &
DeformationModelAssemblerCacheData::scratchForCurrentThread()
{
  return threadScratch_->local();
}
}  // namespace SolidDeformationModel
}  // namespace pgo

DeformationModelAssembler::DeformationModelAssembler(
  std::shared_ptr<DeformationModelManager> dm,
  const Formulation &formulation,
  std::shared_ptr<OptimizableField> elasticParamField,
  std::shared_ptr<OptimizableField> plasticParamField,
  const double *elementWeights_):
  deformationModelManager(std::move(dm)),
  dofLayout(formulation.createDofLayout(*deformationModelManager->getMesh())),
  restDofs_(formulation.buildGlobalRestDofs(*deformationModelManager->getMesh())),
  elasticParamField_(std::move(elasticParamField)),
  plasticParamField_(std::move(plasticParamField))
{
  nele = deformationModelManager->getMesh()->getNumElements();
  neleVtx = deformationModelManager->getMesh()->getNumElementVertices();
  localDOFs = dofLayout->numLocalDofs(0);
  numDOFs = dofLayout->numGlobalDofs();

  numElasticParams_ = deformationModelManager->getDeformationModel(0)->getNumElasticParameters();
  numPlasticParams_ = deformationModelManager->getDeformationModel(0)->getNumPlasticParameters();
  validateParameterField("elasticParamField", elasticParamField_.get(), ParameterDomain::ELASTIC, numElasticParams_, nele);
  validateParameterField("plasticParamField", plasticParamField_.get(), ParameterDomain::PLASTIC, numPlasticParams_, nele);

  const auto *elasticParamLayout = elasticParamField_->dofLayout();
  const auto *plasticParamLayout = plasticParamField_->dofLayout();
  numElasticLocalParams_ = elasticParamLayout ? elasticParamLayout->numLocalDofs() : 0;
  numPlasticLocalParams_ = plasticParamLayout ? plasticParamLayout->numLocalDofs() : 0;

  if (elementWeights_) {
    elementWeights.assign(elementWeights_, elementWeights_ + nele);
  } else {
    elementWeights.assign(nele, 1);
  }

  int maxMaterialLocations = 0;
  for (int i = 0; i < nele; i++) {
    femModels.push_back(deformationModelManager->getDeformationModel(i));
    maxMaterialLocations = std::max(maxMaterialLocations, femModels.back()->getNumMaterialLocations());
  }

  const int maxMaterialParams = std::max(numElasticParams_, numPlasticParams_);
  const int maxLocalParams = std::max(numElasticLocalParams_, numPlasticLocalParams_);
  data = std::make_unique<DeformationModelAssemblerCacheData>(
    localDOFs, maxMaterialLocations, maxMaterialParams, maxLocalParams);
  logMemoryCheckpoint("assembler.after_thread_cache_setup");

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

  // df/db (elastic) template.
  const int numElasticGlobalParams = elasticParamLayout ? elasticParamLayout->numGlobalDofs() : 0;
  if (numElasticParams_ > 0 && numElasticLocalParams_ > 0 && elasticParamLayout) {
    buildMixedSparsityTemplate(
      numElasticLocalParams_, numElasticGlobalParams,
      [elasticParamLayout](int ele, int ep) { return elasticParamLayout->globalDof(ele, ep); },
      dfdbTemplate, element_dfdb_InverseIndices, entries);
  } else {
    dfdbTemplate.resize(numDOFs, 0);
  }

  // df/da (plastic) template.
  const int numPlasticGlobalParams = plasticParamLayout ? plasticParamLayout->numGlobalDofs() : 0;
  if (numPlasticParams_ > 0 && numPlasticLocalParams_ > 0 && plasticParamLayout) {
    buildMixedSparsityTemplate(
      numPlasticLocalParams_, numPlasticGlobalParams,
      [plasticParamLayout](int ele, int pp) { return plasticParamLayout->globalDof(ele, pp); },
      dfdaTemplate, element_dfda_InverseIndices, entries);
  } else {
    dfdaTemplate.resize(numDOFs, 0);
  }

  // d²E/da² (plastic-only) template — involves no displacement DOFs, already generic.
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

    d2Eda2Template.resize(numPlasticGlobalParams, numPlasticGlobalParams);
    d2Eda2Template.setFromTriplets(entries.begin(), entries.end());
  }
  else {
    d2Eda2Template.resize(0, 0);
  }

  element_d2Eda2_InverseIndices.resize(nele);
  if (numPlasticParams_ > 0 && numPlasticLocalParams_ > 0 && plasticParamLayout) {
    for (int ele = 0; ele < nele; ele++) {
      DynamicIndexMatrix idxM(numPlasticLocalParams_, numPlasticLocalParams_);
      idxM.setConstant(-1);

      for (int pi = 0; pi < numPlasticLocalParams_; pi++) {
        const int globalRow = plasticParamLayout->globalDof(ele, pi);
        for (int pj = 0; pj < numPlasticLocalParams_; pj++) {
          const int globalCol = plasticParamLayout->globalDof(ele, pj);
          idxM(pi, pj) = ES::findEntryOffset(d2Eda2Template, globalRow, globalCol);
        }
      }

      element_d2Eda2_InverseIndices[ele] = idxM;
    }
  }

  // d²E/db² (elastic-only).
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

    d2Edb2Template.resize(numElasticGlobalParams, numElasticGlobalParams);
    d2Edb2Template.setFromTriplets(entries.begin(), entries.end());
  }
  else {
    d2Edb2Template.resize(0, 0);
  }

  element_d2Edb2_InverseIndices.resize(nele);
  if (numElasticParams_ > 0 && numElasticLocalParams_ > 0 && elasticParamLayout) {
    for (int ele = 0; ele < nele; ele++) {
      DynamicIndexMatrix idxM(numElasticLocalParams_, numElasticLocalParams_);
      idxM.setConstant(-1);

      for (int pi = 0; pi < numElasticLocalParams_; pi++) {
        const int globalRow = elasticParamLayout->globalDof(ele, pi);
        for (int pj = 0; pj < numElasticLocalParams_; pj++) {
          const int globalCol = elasticParamLayout->globalDof(ele, pj);
          idxM(pi, pj) = ES::findEntryOffset(d2Edb2Template, globalRow, globalCol);
        }
      }

      element_d2Edb2_InverseIndices[ele] = idxM;
    }
  }

  // d²E/da db (plastic rows, elastic columns).
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

    d2EdadbTemplate.resize(numPlasticGlobalParams, numElasticGlobalParams);
    d2EdadbTemplate.setFromTriplets(entries.begin(), entries.end());
  }
  else {
    d2EdadbTemplate.resize(0, 0);
  }

  element_d2Edadb_InverseIndices.resize(nele);
  if (numPlasticParams_ > 0 && numPlasticLocalParams_ > 0 && plasticParamLayout &&
    numElasticParams_ > 0 && numElasticLocalParams_ > 0 && elasticParamLayout) {
    for (int ele = 0; ele < nele; ele++) {
      DynamicIndexMatrix idxM(numPlasticLocalParams_, numElasticLocalParams_);
      idxM.setConstant(-1);

      for (int pi = 0; pi < numPlasticLocalParams_; pi++) {
        const int globalRow = plasticParamLayout->globalDof(ele, pi);
        for (int ej = 0; ej < numElasticLocalParams_; ej++) {
          const int globalCol = elasticParamLayout->globalDof(ele, ej);
          idxM(pi, ej) = ES::findEntryOffset(d2EdadbTemplate, globalRow, globalCol);
        }
      }

      element_d2Edadb_InverseIndices[ele] = idxM;
    }
  }
}

DeformationModelAssembler::~DeformationModelAssembler() = default;

DeformationModelAssembler::PreparedElement DeformationModelAssembler::gatherAndPrepare(
  int ele, const double *x, DeformationModelAssemblerCacheData::ThreadScratch &scratch) const
{
  dofLayout->gather(ele, x, scratch.localPosition.data(), scratch.groups);
  const DeformationModel *fem = femModels[ele];
  DeformationModel::CacheData *cache = scratch.cacheFor(*fem);
  const int numMaterialLocations = fem->getNumMaterialLocations();
  fillElementParamValues(
    elasticParamField_.get(), ele, numMaterialLocations, numElasticParams_, scratch.elasticParamValues.data());
  fillElementParamValues(
    plasticParamField_.get(), ele, numMaterialLocations, numPlasticParams_, scratch.plasticParamValues.data());

  fem->prepareData(scratch.localPosition.data(),
    numElasticParams_ > 0 ? scratch.elasticParamValues.data() : nullptr,
    numPlasticParams_ > 0 ? scratch.plasticParamValues.data() : nullptr,
    cache);
  return { fem, cache };
}

double DeformationModelAssembler::computeEnergy(const double *x) const
{
  for (auto it = data->threadScratch().begin(); it != data->threadScratch().end(); ++it)
    it->energy = 0.0;

  auto localEnergyFunc = [this, x](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->scratchForCurrentThread();
    PreparedElement prepared = gatherAndPrepare(ele, x, scratch);
    double energy = prepared.model->computeEnergy(prepared.cache);

    scratch.energy += energy * elementWeights[ele];
  };

  pgo::parallel::parallelFor(0, nele, localEnergyFunc);

  double energyAll = 0;
  for (auto it = data->threadScratch().begin(); it != data->threadScratch().end(); ++it)
    energyAll += it->energy;

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

    auto &scratch = data->scratchForCurrentThread();
    dofLayout->gather(ele, x, scratch.localPosition.data(), scratch.groups);
    dofLayout->gather(ele, dx, scratch.localDirection.data(), scratch.groups);

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

void DeformationModelAssembler::computeGradient(const double *x, double *grad) const
{
  memset(grad, 0, sizeof(double) * numDOFs);
  auto localGradFunc = [this, x, grad](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->scratchForCurrentThread();
    PreparedElement prepared = gatherAndPrepare(ele, x, scratch);

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

    dofLayout->scatterAddGradient(ele, scratch.localGradient.data(), grad, scratch.groups);
  };

  pgo::parallel::parallelFor(0, nele, localGradFunc);

  if (enableSanityCheck)
    sanityCheckValues(grad, numDOFs, "gradient");
}

void DeformationModelAssembler::computeHessian(const double *x, EigenSupport::SpMatD &hess) const
{
  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  auto localHessFunc = [this, x, &hess](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->scratchForCurrentThread();
    PreparedElement prepared = gatherAndPrepare(ele, x, scratch);

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

  pgo::parallel::parallelFor(0, nele, localHessFunc);

  if (enableSanityCheck)
    sanityCheckValues(hess.valuePtr(), hess.nonZeros(), "Hessian");
}

int DeformationModelAssembler::getNumElasticGlobalParams() const
{
  const auto *layout = elasticParamField_ ? elasticParamField_->dofLayout() : nullptr;
  return layout ? layout->numGlobalDofs() : 0;
}

int DeformationModelAssembler::getNumPlasticGlobalParams() const
{
  const auto *layout = plasticParamField_ ? plasticParamField_->dofLayout() : nullptr;
  return layout ? layout->numGlobalDofs() : 0;
}

ES::VXd DeformationModelAssembler::getElasticParameterSnapshot() const
{
  return snapshot(*elasticParamField_);
}

ES::VXd DeformationModelAssembler::getPlasticParameterSnapshot() const
{
  return snapshot(*plasticParamField_);
}

void DeformationModelAssembler::setElasticValues(ES::ConstRefVecXd values)
{
  setFieldValues(*elasticParamField_, values, "DeformationModelAssembler::setElasticValues");
}

void DeformationModelAssembler::setPlasticValues(ES::ConstRefVecXd values)
{
  setFieldValues(*plasticParamField_, values, "DeformationModelAssembler::setPlasticValues");
}

void DeformationModelAssembler::computePlasticGradient(const double *x, double *grad) const
{
  const int numPlasticGlobalParams = getNumPlasticGlobalParams();
  std::fill(grad, grad + numPlasticGlobalParams, 0.0);

  if (numPlasticParams_ == 0 || numPlasticLocalParams_ == 0 || numPlasticGlobalParams == 0)
    return;

  const auto *plasticParamLayout = plasticParamField_ ? plasticParamField_->dofLayout() : nullptr;
  if (!plasticParamLayout)
    return;

  auto localGradFunc = [this, x, grad, plasticParamLayout](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->scratchForCurrentThread();
    PreparedElement prepared = gatherAndPrepare(ele, x, scratch);

    Eigen::Map<ES::VXd> rawGrad(scratch.rawParamGradient.data(), numPlasticParams_);
    rawGrad.setZero();
    prepared.model->compute_dE_da(prepared.cache, rawGrad.data());
    fillLocalParamDerivative(*plasticParamField_, ele, 0, numPlasticParams_, numPlasticLocalParams_,
      scratch.paramDerivativeData.data());
    const Eigen::Map<const ES::MXd> dParamDLocal(
      scratch.paramDerivativeData.data(), numPlasticParams_, numPlasticLocalParams_);
    scratch.localParamGradient.head(numPlasticLocalParams_).noalias() = dParamDLocal.transpose() * rawGrad;
    scratch.localParamGradient.head(numPlasticLocalParams_) *= elementWeights[ele];

    for (int pi = 0; pi < numPlasticLocalParams_; pi++) {
      const int globalRow = plasticParamLayout->globalDof(ele, pi);
      std::atomic_ref<double> gradRef(grad[globalRow]);
      gradRef.fetch_add(scratch.localParamGradient[pi]);
    }
  };

  pgo::parallel::parallelFor(0, nele, localGradFunc);

  if (enableSanityCheck)
    sanityCheckValues(grad, numPlasticGlobalParams, "plastic gradient");
}

void DeformationModelAssembler::computePlasticHessian(const double *x, EigenSupport::SpMatD &hess) const
{
  if (hess.rows() != d2Eda2Template.rows() || hess.cols() != d2Eda2Template.cols() ||
    hess.nonZeros() != d2Eda2Template.nonZeros()) {
    hess = d2Eda2Template;
  }

  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  const int numPlasticGlobalParams = getNumPlasticGlobalParams();
  if (numPlasticParams_ == 0 || numPlasticLocalParams_ == 0 || numPlasticGlobalParams == 0)
    return;

  auto localHessFunc = [this, x, &hess](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->scratchForCurrentThread();
    PreparedElement prepared = gatherAndPrepare(ele, x, scratch);

    prepared.model->compute_d2E_da2(prepared.cache, scratch.localMatrixData.data());

    const ES::Mp<ES::MXd> rawH(scratch.localMatrixData.data(), numPlasticParams_, numPlasticParams_);
    fillLocalParamDerivative(*plasticParamField_, ele, 0, numPlasticParams_, numPlasticLocalParams_,
      scratch.paramDerivativeData.data());
    const Eigen::Map<const ES::MXd> dParamDLocal(
      scratch.paramDerivativeData.data(), numPlasticParams_, numPlasticLocalParams_);
    auto paramWork = scratch.paramWorkMatrix.block(0, 0, numPlasticParams_, numPlasticLocalParams_);
    auto localH = scratch.localParamHessian.block(0, 0, numPlasticLocalParams_, numPlasticLocalParams_);
    paramWork.noalias() = rawH * dParamDLocal;
    localH.noalias() = dParamDLocal.transpose() * paramWork;
    localH *= elementWeights[ele];

    const auto &idxM = element_d2Eda2_InverseIndices[ele];
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

  pgo::parallel::parallelFor(0, nele, localHessFunc);

  if (enableSanityCheck)
    sanityCheckValues(hess.valuePtr(), hess.nonZeros(), "plastic Hessian");
}

void DeformationModelAssembler::computeElasticGradient(const double *x, double *grad) const
{
  const int numElasticGlobalParams = getNumElasticGlobalParams();
  std::fill(grad, grad + numElasticGlobalParams, 0.0);

  if (numElasticParams_ == 0 || numElasticLocalParams_ == 0 || numElasticGlobalParams == 0)
    return;

  const auto *elasticParamLayout = elasticParamField_ ? elasticParamField_->dofLayout() : nullptr;
  if (!elasticParamLayout)
    return;

  auto localGradFunc = [this, x, grad, elasticParamLayout](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->scratchForCurrentThread();
    PreparedElement prepared = gatherAndPrepare(ele, x, scratch);

    Eigen::Map<ES::VXd> rawGrad(scratch.rawParamGradient.data(), numElasticParams_);
    rawGrad.setZero();
    prepared.model->compute_dE_db(prepared.cache, rawGrad.data());
    fillLocalParamDerivative(*elasticParamField_, ele, 0, numElasticParams_, numElasticLocalParams_,
      scratch.paramDerivativeData.data());
    const Eigen::Map<const ES::MXd> dParamDLocal(
      scratch.paramDerivativeData.data(), numElasticParams_, numElasticLocalParams_);
    scratch.localParamGradient.head(numElasticLocalParams_).noalias() = dParamDLocal.transpose() * rawGrad;
    scratch.localParamGradient.head(numElasticLocalParams_) *= elementWeights[ele];

    for (int pi = 0; pi < numElasticLocalParams_; pi++) {
      const int globalRow = elasticParamLayout->globalDof(ele, pi);
      std::atomic_ref<double> gradRef(grad[globalRow]);
      gradRef.fetch_add(scratch.localParamGradient[pi]);
    }
  };

  pgo::parallel::parallelFor(0, nele, localGradFunc);

  if (enableSanityCheck)
    sanityCheckValues(grad, numElasticGlobalParams, "elastic gradient");
}

void DeformationModelAssembler::computeElasticHessian(const double *x, EigenSupport::SpMatD &hess) const
{
  if (hess.rows() != d2Edb2Template.rows() || hess.cols() != d2Edb2Template.cols() ||
    hess.nonZeros() != d2Edb2Template.nonZeros()) {
    hess = d2Edb2Template;
  }

  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  const int numElasticGlobalParams = getNumElasticGlobalParams();
  if (numElasticParams_ == 0 || numElasticLocalParams_ == 0 || numElasticGlobalParams == 0)
    return;

  auto localHessFunc = [this, x, &hess](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->scratchForCurrentThread();
    PreparedElement prepared = gatherAndPrepare(ele, x, scratch);

    prepared.model->compute_d2E_db2(prepared.cache, scratch.localMatrixData.data());

    const ES::Mp<ES::MXd> rawH(scratch.localMatrixData.data(), numElasticParams_, numElasticParams_);
    fillLocalParamDerivative(*elasticParamField_, ele, 0, numElasticParams_, numElasticLocalParams_,
      scratch.paramDerivativeData.data());
    const Eigen::Map<const ES::MXd> dParamDLocal(
      scratch.paramDerivativeData.data(), numElasticParams_, numElasticLocalParams_);
    auto paramWork = scratch.paramWorkMatrix.block(0, 0, numElasticParams_, numElasticLocalParams_);
    auto localH = scratch.localParamHessian.block(0, 0, numElasticLocalParams_, numElasticLocalParams_);
    paramWork.noalias() = rawH * dParamDLocal;
    localH.noalias() = dParamDLocal.transpose() * paramWork;
    localH *= elementWeights[ele];

    const auto &idxM = element_d2Edb2_InverseIndices[ele];
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

  pgo::parallel::parallelFor(0, nele, localHessFunc);

  if (enableSanityCheck)
    sanityCheckValues(hess.valuePtr(), hess.nonZeros(), "elastic Hessian");
}

void DeformationModelAssembler::computePlasticElasticHessian(const double *x, EigenSupport::SpMatD &hess) const
{
  if (hess.rows() != d2EdadbTemplate.rows() || hess.cols() != d2EdadbTemplate.cols() ||
    hess.nonZeros() != d2EdadbTemplate.nonZeros()) {
    hess = d2EdadbTemplate;
  }

  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  const int numPlasticGlobalParams = getNumPlasticGlobalParams();
  const int numElasticGlobalParams = getNumElasticGlobalParams();
  if (numPlasticParams_ == 0 || numPlasticLocalParams_ == 0 || numPlasticGlobalParams == 0 ||
    numElasticParams_ == 0 || numElasticLocalParams_ == 0 || numElasticGlobalParams == 0)
    return;

  auto localHessFunc = [this, x, &hess](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->scratchForCurrentThread();
    PreparedElement prepared = gatherAndPrepare(ele, x, scratch);

    prepared.model->compute_d2E_dadb(prepared.cache, scratch.localMatrixData.data());

    const ES::Mp<ES::MXd> rawH(scratch.localMatrixData.data(), numPlasticParams_, numElasticParams_);
    fillLocalParamDerivative(*plasticParamField_, ele, 0, numPlasticParams_, numPlasticLocalParams_,
      scratch.paramDerivativeData.data());
    fillLocalParamDerivative(*elasticParamField_, ele, 0, numElasticParams_, numElasticLocalParams_,
      scratch.paramDerivativeData2.data());
    const Eigen::Map<const ES::MXd> dPlasticDLocal(
      scratch.paramDerivativeData.data(), numPlasticParams_, numPlasticLocalParams_);
    const Eigen::Map<const ES::MXd> dElasticDLocal(
      scratch.paramDerivativeData2.data(), numElasticParams_, numElasticLocalParams_);

    auto paramWork = scratch.paramWorkMatrix.block(0, 0, numPlasticParams_, numElasticLocalParams_);
    auto localH = scratch.localParamHessian.block(0, 0, numPlasticLocalParams_, numElasticLocalParams_);
    paramWork.noalias() = rawH * dElasticDLocal;
    localH.noalias() = dPlasticDLocal.transpose() * paramWork;
    localH *= elementWeights[ele];

    const auto &idxM = element_d2Edadb_InverseIndices[ele];
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

  pgo::parallel::parallelFor(0, nele, localHessFunc);

  if (enableSanityCheck)
    sanityCheckValues(hess.valuePtr(), hess.nonZeros(), "plastic-elastic Hessian");
}

void DeformationModelAssembler::compute_df_da(const double *x, EigenSupport::SpMatD &hess) const
{
  if (numPlasticParams_ == 0)
    return;
  assembleDfDparam(x, numPlasticParams_, numPlasticLocalParams_, plasticParamField_.get(),
                   element_dfda_InverseIndices,
                   &DeformationModel::compute_d2E_dxda, hess, "df/da");
}

void DeformationModelAssembler::compute_df_db(const double *x, EigenSupport::SpMatD &hess) const
{
  if (numElasticParams_ == 0)
    return;
  assembleDfDparam(x, numElasticParams_, numElasticLocalParams_, elasticParamField_.get(),
                   element_dfdb_InverseIndices,
                   &DeformationModel::compute_d2E_dxdb, hess, "df/db");
}

void DeformationModelAssembler::computeVonMisesStresses(const double *x, double *elementStresses) const
{
  std::fill(elementStresses, elementStresses + nele, 0.0);

  auto localStressFunc = [this, x, elementStresses](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->scratchForCurrentThread();
    PreparedElement prepared = gatherAndPrepare(ele, x, scratch);

    int nPt = 0;
    std::fill(scratch.materialLocationValues.begin(), scratch.materialLocationValues.end(), 0.0);
    prepared.model->vonMisesStress(prepared.cache, nPt, scratch.materialLocationValues.data());
    if (nPt <= 0) {
      elementStresses[ele] = 0.0;
      return;
    }

    const int stressCount = std::min<int>(nPt, static_cast<int>(scratch.materialLocationValues.size()));
    elementStresses[ele] = *std::max_element(
      scratch.materialLocationValues.begin(), scratch.materialLocationValues.begin() + stressCount);
  };

  pgo::parallel::parallelFor(0, nele, localStressFunc);
}

void DeformationModelAssembler::computeMaxStrains(const double *x, double *elementStrain) const
{
  std::fill(elementStrain, elementStrain + nele, 0.0);

  auto localStrainFunc = [this, x, elementStrain](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->scratchForCurrentThread();
    PreparedElement prepared = gatherAndPrepare(ele, x, scratch);

    int nPt = 0;
    std::fill(scratch.materialLocationValues.begin(), scratch.materialLocationValues.end(), 0.0);
    prepared.model->maxStrain(prepared.cache, nPt, scratch.materialLocationValues.data());
    if (nPt <= 0) {
      elementStrain[ele] = 0.0;
      return;
    }

    const int strainCount = std::min<int>(nPt, static_cast<int>(scratch.materialLocationValues.size()));
    elementStrain[ele] = *std::max_element(
      scratch.materialLocationValues.begin(), scratch.materialLocationValues.begin() + strainCount);
  };

  pgo::parallel::parallelFor(0, nele, localStrainFunc);
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

void DeformationModelAssembler::assembleDfDparam(
  const double *x,
  int numMaterialParams,
  int numLocalParams,
  const OptimizableField *paramField,
  const std::vector<DynamicIndexMatrix> &inverseIndices,
  void (DeformationModel::*computeLocal)(const DeformationModel::CacheData *, double *) const,
  EigenSupport::SpMatD &hess,
  const char *label) const
{
  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  if (!paramField || numMaterialParams == 0 || numLocalParams == 0)
    return;

  auto localFunc = [this, x, &hess, numMaterialParams, numLocalParams, paramField, &inverseIndices, computeLocal](int ele) {
    if (elementWeights[ele] == 0)
      return;

    auto &scratch = data->scratchForCurrentThread();
    PreparedElement prepared = gatherAndPrepare(ele, x, scratch);

    (prepared.model->*computeLocal)(prepared.cache, scratch.localMatrixData.data());

    const ES::Mp<ES::MXd> rawK(scratch.localMatrixData.data(), localDOFs, numMaterialParams);
    fillLocalParamDerivative(*paramField, ele, 0, numMaterialParams, numLocalParams,
      scratch.paramDerivativeData.data());
    const Eigen::Map<const ES::MXd> dParamDLocal(
      scratch.paramDerivativeData.data(), numMaterialParams, numLocalParams);
    auto localK = scratch.localMixedMatrix.block(0, 0, localDOFs, numLocalParams);
    localK.noalias() = rawK * dParamDLocal;
    localK *= elementWeights[ele];

    const auto &idxM = inverseIndices[ele];
    for (int localRow = 0; localRow < localDOFs; localRow++) {
      for (int va = 0; va < numLocalParams; va++) {
        std::ptrdiff_t offset = idxM(localRow, va);
        if (offset >= 0) {
          std::atomic_ref<double> hessRef(hess.valuePtr()[offset]);
          hessRef.fetch_add(localK(localRow, va));
        }
      }
    }
  };

  for (int ele = 0; ele < nele; ele++) {
    localFunc(ele);
  }

  if (enableSanityCheck)
    sanityCheckValues(hess.valuePtr(), hess.nonZeros(), label);
}
