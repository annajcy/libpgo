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

#include <tbb/parallel_for.h>
#include <tbb/enumerable_thread_specific.h>

#include <algorithm>
#include <atomic>
#include <stdexcept>
#include <string>

using namespace pgo::SolidDeformationModel;

namespace ES = pgo::EigenSupport;

namespace
{
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

ES::MXd computeLocalParamDerivative(
  const OptimizableField &field, int ele, int quadratureId, int numChannels, int numLocalDofs)
{
  ES::MXd deriv = ES::MXd::Zero(numChannels, numLocalDofs);
  if (numChannels > 0 && numLocalDofs > 0) {
    field.computeDerivative(ele, quadratureId, deriv.data());
  }
  return deriv;
}

ES::VXd computeElementParamValues(
  const OptimizableField *field, int ele, int numMaterialLocations, int numChannels)
{
  if (!field || numChannels == 0 || numMaterialLocations == 0) {
    return ES::VXd();
  }

  ES::VXd values(numMaterialLocations * numChannels);
  for (int q = 0; q < numMaterialLocations; q++) {
    field->computeValue(ele, q, values.data() + static_cast<std::ptrdiff_t>(q) * numChannels);
  }
  return values;
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
}

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

  data = std::make_unique<DeformationModelAssemblerCacheData>();

  for (int i = 0; i < nele; i++) {
    femModels.push_back(deformationModelManager->getDeformationModel(i));
    data->elementCacheData.push_back(femModels.back()->allocateCacheData());
  }

  SPDLOG_LOGGER_INFO(Logging::lgr(), "Assembler parameter channels:{},{} local:{},{}",
    numElasticParams_, numPlasticParams_, numElasticLocalParams_, numPlasticLocalParams_);

  // Hessian template.
  std::vector<ES::TripletD> entries;
  for (int ele = 0; ele < nele; ele++) {
    dofLayout->addHessianSparsity(ele, entries);
  }

  KTemplate.resize(numDOFs, numDOFs);
  KTemplate.setFromTriplets(entries.begin(), entries.end());

  elementKInverseIndices.resize(nele);
  for (int ele = 0; ele < nele; ele++) {
    DynamicIndexMatrix idxM;
    dofLayout->buildLocalToGlobalMatrixIndices(ele, KTemplate, idxM);
    elementKInverseIndices[ele] = idxM;
  }

  // NOTE (tricubic Hermite seam): the d2E/da2 template and inverse-index map below still only
  // involve plastic DOFs (no displacement indices), so it is already generic across formulations.
  // The df/db and df/da templates were generalized in phase 3 to use dofLayout->getGlobalDofIndices,
  // matching the computeHessian pattern.
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
}

DeformationModelAssembler::~DeformationModelAssembler() = default;

const DeformationModel *DeformationModelAssembler::gatherAndPrepare(
  int ele, const double *x, double *localBuf) const
{
  dofLayout->gather(ele, x, localBuf);
  const DeformationModel *fem = femModels[ele];
  const int numMaterialLocations = fem->getNumMaterialLocations();
  const ES::VXd elasticParams =
    computeElementParamValues(elasticParamField_.get(), ele, numMaterialLocations, numElasticParams_);
  const ES::VXd plasticParams =
    computeElementParamValues(plasticParamField_.get(), ele, numMaterialLocations, numPlasticParams_);

  fem->prepareData(localBuf,
    elasticParams.size() ? elasticParams.data() : nullptr,
    plasticParams.size() ? plasticParams.data() : nullptr,
    data->elementCacheData[ele].get());
  return fem;
}

double DeformationModelAssembler::computeEnergy(const double *x) const
{
  for (auto it = data->energyLocalBuffer.begin(); it != data->energyLocalBuffer.end(); ++it)
    *it = 0.0;

  auto localEnergyFunc = [this, x](int ele) {
    if (elementWeights[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    const DeformationModel *fem = gatherAndPrepare(ele, x, localp.data());
    double energy = fem->computeEnergy(data->elementCacheData[ele].get());

    data->energyLocalBuffer.local() += energy * elementWeights[ele];
  };

  tbb::parallel_for(0, nele, localEnergyFunc, data->partitioners[0]);

  double energyAll = 0;
  for (auto it = data->energyLocalBuffer.begin(); it != data->energyLocalBuffer.end(); ++it)
    energyAll += *it;

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

    ES::VXd localX(localDOFs);
    ES::VXd localDx(localDOFs);
    dofLayout->gather(ele, x, localX.data());
    dofLayout->gather(ele, dx, localDx.data());

    const DeformationModel::LocalMaxStepResult localResult = femModels[ele]->computeLocalMaxStepSize(localX.data(), localDx.data());
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

    ES::VXd localp(localDOFs);
    const DeformationModel *fem = gatherAndPrepare(ele, x, localp.data());

    ES::VXd localGradx(localDOFs);
    fem->compute_dE_dx(data->elementCacheData[ele].get(), localGradx.data());
    localGradx *= elementWeights[ele];

    if (enableSanityCheck) {
      for (int i = 0; i < localDOFs; i++) {
        if (std::isfinite(localGradx[i]) == false) {
          SPDLOG_LOGGER_ERROR(Logging::lgr(), "Ele: {}", ele);
          SPDLOG_LOGGER_ERROR(Logging::lgr(), "Encounter weird numbers.\nGrad:\n{}\n;x:{}\n", localGradx, localp);
        }
      }
    }

    dofLayout->scatterAddGradient(ele, localGradx.data(), grad);
  };

  tbb::parallel_for(0, nele, localGradFunc, data->partitioners[1]);

  if (enableSanityCheck)
    sanityCheckValues(grad, numDOFs, "gradient");
}

void DeformationModelAssembler::computeHessian(const double *x, EigenSupport::SpMatD &hess) const
{
  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  auto localHessFunc = [this, x, &hess](int ele) {
    if (elementWeights[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    const DeformationModel *fem = gatherAndPrepare(ele, x, localp.data());

    std::vector<double> localKData(localDOFs * localDOFs);
    fem->compute_d2E_dx2(data->elementCacheData[ele].get(), localKData.data());

    ES::Mp<ES::MXd> localK(localKData.data(), localDOFs, localDOFs);
    localK *= elementWeights[ele];

    const auto &idxM = elementKInverseIndices[ele];

    std::vector<int> globalDofIndices;
    dofLayout->getGlobalDofIndices(ele, globalDofIndices);

    // Generic over the layout's local DOF count: idxM and globalDofIndices are both sized to
    // localDOFs by the DofLayout, so this fills the whole local stiffness block regardless of how
    // many DOFs each node carries. For Vertex3 (localDOFs == neleVtx*3) this visits exactly the
    // same (row, col) pairs as the old per-vertex nest; for tricubic Hermite it assembles the full
    // 192x192 instead of only the first 24x24.
    for (int localRow = 0; localRow < localDOFs; localRow++) {
      if (globalDofIndices[localRow] < 0)
        continue;
      for (int localCol = 0; localCol < localDOFs; localCol++) {
        if (globalDofIndices[localCol] < 0)
          continue;
        std::ptrdiff_t offset = idxM(localRow, localCol);
        if (offset >= 0) {
          std::atomic_ref<double> hessRef(hess.valuePtr()[offset]);
          hessRef.fetch_add(localK(localRow, localCol));
        }
      }
    }
  };

  tbb::parallel_for(0, nele, localHessFunc, data->partitioners[2]);

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

    ES::VXd localp(localDOFs);
    const DeformationModel *fem = gatherAndPrepare(ele, x, localp.data());

    ES::VXd rawGrad = ES::VXd::Zero(numPlasticParams_);
    fem->compute_dE_da(data->elementCacheData[ele].get(), rawGrad.data());
    const ES::MXd dParamDLocal =
      computeLocalParamDerivative(*plasticParamField_, ele, 0, numPlasticParams_, numPlasticLocalParams_);
    ES::VXd localGrad = dParamDLocal.transpose() * rawGrad;
    localGrad *= elementWeights[ele];

    for (int pi = 0; pi < numPlasticLocalParams_; pi++) {
      const int globalRow = plasticParamLayout->globalDof(ele, pi);
      std::atomic_ref<double> gradRef(grad[globalRow]);
      gradRef.fetch_add(localGrad[pi]);
    }
  };

  tbb::parallel_for(0, nele, localGradFunc, data->partitioners[0]);

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

    ES::VXd localp(localDOFs);
    const DeformationModel *fem = gatherAndPrepare(ele, x, localp.data());

    std::vector<double> localHData(numPlasticParams_ * numPlasticParams_);
    fem->compute_d2E_da2(data->elementCacheData[ele].get(), localHData.data());

    const ES::Mp<ES::MXd> rawH(localHData.data(), numPlasticParams_, numPlasticParams_);
    const ES::MXd dParamDLocal =
      computeLocalParamDerivative(*plasticParamField_, ele, 0, numPlasticParams_, numPlasticLocalParams_);
    ES::MXd localH = dParamDLocal.transpose() * rawH * dParamDLocal;
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

  tbb::parallel_for(0, nele, localHessFunc, data->partitioners[1]);

  if (enableSanityCheck)
    sanityCheckValues(hess.valuePtr(), hess.nonZeros(), "plastic Hessian");
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

    ES::VXd localp(localDOFs);
    const DeformationModel *fem = gatherAndPrepare(ele, x, localp.data());

    int nPt = 0;
    std::vector<double> localStresses(std::max(16, fem->getNumMaterialLocations()), 0.0);
    fem->vonMisesStress(data->elementCacheData[ele].get(), nPt, localStresses.data());
    if (nPt <= 0) {
      elementStresses[ele] = 0.0;
      return;
    }

    const int stressCount = std::min<int>(nPt, static_cast<int>(localStresses.size()));
    elementStresses[ele] = *std::max_element(localStresses.begin(), localStresses.begin() + stressCount);
  };

  tbb::parallel_for(0, nele, localStressFunc, data->partitioners[3]);
}

void DeformationModelAssembler::computeMaxStrains(const double *x, double *elementStrain) const
{
  std::fill(elementStrain, elementStrain + nele, 0.0);

  auto localStrainFunc = [this, x, elementStrain](int ele) {
    if (elementWeights[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    const DeformationModel *fem = gatherAndPrepare(ele, x, localp.data());

    int nPt = 0;
    std::vector<double> localStrains(std::max(16, fem->getNumMaterialLocations()), 0.0);
    fem->maxStrain(data->elementCacheData[ele].get(), nPt, localStrains.data());
    if (nPt <= 0) {
      elementStrain[ele] = 0.0;
      return;
    }

    const int strainCount = std::min<int>(nPt, static_cast<int>(localStrains.size()));
    elementStrain[ele] = *std::max_element(localStrains.begin(), localStrains.begin() + strainCount);
  };

  tbb::parallel_for(0, nele, localStrainFunc, data->partitioners[4]);
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
    std::vector<int> globalDofIndices;
    dofLayout->getGlobalDofIndices(ele, globalDofIndices);
    for (int localRow = 0; localRow < localDOFs; localRow++) {
      int globalRow = globalDofIndices[localRow];
      if (globalRow < 0) continue;
      for (int pp = 0; pp < numLocalParams; pp++)
        entries.emplace_back(globalRow, paramGlobalCol(ele, pp), 1.0);
    }
  }
  tmpl.resize(numDOFs, numGlobalParams);
  tmpl.setFromTriplets(entries.begin(), entries.end());

  inverseIndices.resize(nele);
  for (int ele = 0; ele < nele; ele++) {
    DynamicIndexMatrix idxM(localDOFs, numLocalParams);
    idxM.setConstant(-1);

    std::vector<int> globalDofIndices;
    dofLayout->getGlobalDofIndices(ele, globalDofIndices);
    for (int localRow = 0; localRow < localDOFs; localRow++) {
      int globalRow = globalDofIndices[localRow];
      if (globalRow < 0) continue;
      for (int pp = 0; pp < numLocalParams; pp++) {
        int globalCol = paramGlobalCol(ele, pp);
        idxM(localRow, pp) = ES::findEntryOffset(tmpl, globalRow, globalCol);
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

    ES::VXd localp(localDOFs);
    const DeformationModel *fem = gatherAndPrepare(ele, x, localp.data());

    std::vector<double> localKData(localDOFs * numMaterialParams);
    (fem->*computeLocal)(data->elementCacheData[ele].get(), localKData.data());

    const ES::Mp<ES::MXd> rawK(localKData.data(), localDOFs, numMaterialParams);
    const ES::MXd dParamDLocal =
      computeLocalParamDerivative(*paramField, ele, 0, numMaterialParams, numLocalParams);
    ES::MXd localK = rawK * dParamDLocal;
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
