/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "deformationModelAssembler.h"
#include "deformationModelAssemblerCacheData.h"
#include "materialMaxStepPolynomialUtils.h"
#include "deformationModelManager.h"
#include "simulationMesh.h"
#include "deformationModel.h"
#include "elasticModel.h"
#include "plasticModel.h"
#include "formulations/parameters/parameterField.h"

#include "pgoLogging.h"
#include "EigenSupport.h"
#include "fmtEigen.h"

#include <tbb/parallel_for.h>
#include <tbb/enumerable_thread_specific.h>

#include <algorithm>
#include <atomic>

using namespace pgo::SolidDeformationModel;

namespace ES = pgo::EigenSupport;

namespace
{
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
}

DeformationModelAssembler::DeformationModelAssembler(
  std::unique_ptr<DeformationModelManager> dm,
  const double *elementWeights_):
  deformationModelManager(std::move(dm)),
  dofLayout(deformationModelManager->createDofLayout()),
  elasticParamField_(deformationModelManager->getElasticParameterField()),
  plasticParamField_(deformationModelManager->getPlasticParameterField())
{
  nele = deformationModelManager->getMesh()->getNumElements();
  neleVtx = deformationModelManager->getMesh()->getNumElementVertices();
  localDOFs = dofLayout->numLocalDofs(0);
  numDOFs = dofLayout->numGlobalDofs();

  numElasticParams_ = deformationModelManager->getDeformationModel(0)->getElasticModel()->getNumParameters();
  numPlasticParams_ = deformationModelManager->getDeformationModel(0)->getPlasticModel()->getNumParameters();

  if (elementWeights_) {
    elementWeights.assign(elementWeights_, elementWeights_ + nele);
  }
  else {
    elementWeights.assign(nele, 1);
  }

  data = std::make_unique<DeformationModelAssemblerCacheData>();

  for (int i = 0; i < nele; i++) {
    femModels.push_back(deformationModelManager->getDeformationModel(i));
    data->elementCacheData.push_back(femModels.back()->allocateCacheData());
  }

  SPDLOG_LOGGER_INFO(Logging::lgr(), "Assembler parameter:{},{}", numElasticParams_, numPlasticParams_);

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

  // dfdbTemplate: use the field's DOF layout for sparsity.
  entries.clear();
  if (numElasticParams_ > 0 && elasticParamField_) {
    const auto *optField = dynamic_cast<const OptimizableField *>(elasticParamField_);
    if (optField) {
      for (int ele = 0; ele < nele; ele++) {
        for (int vi = 0; vi < neleVtx; vi++) {
          int vidx = deformationModelManager->getMesh()->getVertexIndex(ele, vi);
          if (vidx < 0) continue;
          for (int dof = 0; dof < 3; dof++) {
            int globalRow = vidx * 3 + dof;
            for (int ep = 0; ep < numElasticParams_; ep++)
              entries.emplace_back(globalRow, ele * numElasticParams_ + ep, 1.0);
          }
        }
      }
    }
    dfdbTemplate.resize(numDOFs, nele * numElasticParams_);
    dfdbTemplate.setFromTriplets(entries.begin(), entries.end());
  } else {
    dfdbTemplate.resize(numDOFs, 0);
  }

  element_dfdb_InverseIndices.resize(nele);
  if (numElasticParams_ > 0) {
    for (int ele = 0; ele < nele; ele++) {
      const int *vertexIndices = deformationModelManager->getMesh()->getVertexIndices(ele);
      DynamicIndexMatrix idxM(localDOFs, numElasticParams_);
      idxM.setConstant(-1);

      for (int vi = 0; vi < neleVtx; vi++) {
        for (int dofi = 0; dofi < 3; dofi++) {
          for (int ep = 0; ep < numElasticParams_; ep++) {
            int localRow = vi * 3 + dofi;
            int localCol = ep;

            if (vertexIndices[vi] >= 0) {
              int globalRow = vertexIndices[vi] * 3 + dofi;
              int globalCol = ele * numElasticParams_ + ep;

              idxM(localRow, localCol) = ES::findEntryOffset(dfdbTemplate, globalRow, globalCol);
            }
            else {
              idxM(localRow, localCol) = -1;
            }
          }
        }
      }

      element_dfdb_InverseIndices[ele] = idxM;
    }
  }

  // dfdaTemplate: use the field's DOF layout for sparsity.
  entries.clear();
  if (numPlasticParams_ > 0 && plasticParamField_) {
    const auto *optField = dynamic_cast<const OptimizableField *>(plasticParamField_);
    if (optField) {
      for (int ele = 0; ele < nele; ele++) {
        for (int vi = 0; vi < neleVtx; vi++) {
          int vidx = deformationModelManager->getMesh()->getVertexIndex(ele, vi);
          if (vidx < 0) continue;
          for (int dof = 0; dof < 3; dof++) {
            int globalRow = vidx * 3 + dof;
            for (int pp = 0; pp < numPlasticParams_; pp++)
              entries.emplace_back(globalRow, ele * numPlasticParams_ + pp, 1.0);
          }
        }
      }
    }
    dfdaTemplate.resize(numDOFs, nele * numPlasticParams_);
    dfdaTemplate.setFromTriplets(entries.begin(), entries.end());
  } else {
    dfdaTemplate.resize(numDOFs, 0);
  }

  element_dfda_InverseIndices.resize(nele);
  if (numPlasticParams_ > 0) {
    for (int ele = 0; ele < nele; ele++) {
      const int *vertexIndices = deformationModelManager->getMesh()->getVertexIndices(ele);
      DynamicIndexMatrix idxM(localDOFs, numPlasticParams_);
      idxM.setConstant(-1);

      for (int vi = 0; vi < neleVtx; vi++) {
        for (int dofi = 0; dofi < 3; dofi++) {
          for (int pp = 0; pp < numPlasticParams_; pp++) {
            int localRow = vi * 3 + dofi;
            int localCol = pp;

            if (vertexIndices[vi] >= 0) {
              int globalRow = vertexIndices[vi] * 3 + dofi;
              int globalCol = ele * numPlasticParams_ + pp;

              idxM(localRow, localCol) = ES::findEntryOffset(dfdaTemplate, globalRow, globalCol);
            }
            else {
              idxM(localRow, localCol) = -1;
            }
          }
        }
      }

      element_dfda_InverseIndices[ele] = idxM;
    }
  }
}

DeformationModelAssembler::~DeformationModelAssembler() = default;

double DeformationModelAssembler::computeEnergy(const double *x) const
{
  for (auto it = data->energyLocalBuffer.begin(); it != data->energyLocalBuffer.end(); ++it)
    *it = 0.0;

  auto localEnergyFunc = [this, x](int ele) {
    if (elementWeights[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    dofLayout->gather(ele, x, localp.data());

    const DeformationModel *fem = femModels[ele];
    fem->prepareData(localp.data(), data->elementCacheData[ele].get());
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
    dofLayout->gather(ele, x, localp.data());

    const DeformationModel *fem = femModels[ele];
    fem->prepareData(localp.data(), data->elementCacheData[ele].get());

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

  if (enableSanityCheck) {
    for (int i = 0; i < numDOFs; i++) {
      int fpclass = std::fpclassify(grad[i]);
      if (fpclass == FP_INFINITE || fpclass == FP_NAN) {
        SPDLOG_LOGGER_ERROR(Logging::lgr(), "Encounter weird numbers at {}: {}", i, grad[i]);
        throw std::logic_error("Encounter weird numbers.");
      }
      else if (fpclass == FP_SUBNORMAL) {
        grad[i] = 0;
      }
    }
  }
}

void DeformationModelAssembler::computeHessian(const double *x, EigenSupport::SpMatD &hess) const
{
  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  auto localHessFunc = [this, x, &hess](int ele) {
    if (elementWeights[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    dofLayout->gather(ele, x, localp.data());

    const DeformationModel *fem = femModels[ele];
    fem->prepareData(localp.data(), data->elementCacheData[ele].get());

    std::vector<double> localKData(localDOFs * localDOFs);
    fem->compute_d2E_dx2(data->elementCacheData[ele].get(), localKData.data());

    ES::Mp<ES::MXd> localK(localKData.data(), localDOFs, localDOFs);
    localK *= elementWeights[ele];

    const auto &idxM = elementKInverseIndices[ele];

    std::vector<int> globalDofIndices;
    dofLayout->getGlobalDofIndices(ele, globalDofIndices);

    for (int vi = 0; vi < neleVtx; vi++) {
      if (globalDofIndices[vi * 3] < 0)
        continue;
      for (int vj = 0; vj < neleVtx; vj++) {
        if (globalDofIndices[vj * 3] < 0)
          continue;
        for (int dofi = 0; dofi < 3; dofi++) {
          for (int dofj = 0; dofj < 3; dofj++) {
            int localRow = vi * 3 + dofi;
            int localCol = vj * 3 + dofj;
            std::ptrdiff_t offset = idxM(localRow, localCol);
            if (offset >= 0) {
              std::atomic_ref<double> hessRef(hess.valuePtr()[offset]);
              hessRef.fetch_add(localK(localRow, localCol));
            }
          }
        }
      }
    }
  };

  tbb::parallel_for(0, nele, localHessFunc, data->partitioners[2]);

  if (enableSanityCheck) {
    for (Eigen::Index i = 0; i < hess.nonZeros(); i++) {
      int fpclass = std::fpclassify(hess.valuePtr()[i]);
      if (fpclass == FP_INFINITE || fpclass == FP_NAN) {
        SPDLOG_LOGGER_ERROR(Logging::lgr(), "Encounter weird numbers at {}: {}", i, hess.valuePtr()[i]);
        throw std::logic_error("Encounter weird numbers.");
      }
      else if (fpclass == FP_SUBNORMAL) {
        hess.valuePtr()[i] = 0;
      }
    }
  }
}

void DeformationModelAssembler::compute_df_da(const double *x, EigenSupport::SpMatD &hess) const
{
  if (numPlasticParams_ == 0)
    return;

  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  auto localHessFunc = [this, x, &hess](int ele) {
    if (elementWeights[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    dofLayout->gather(ele, x, localp.data());

    const DeformationModel *fem = femModels[ele];
    fem->prepareData(localp.data(), data->elementCacheData[ele].get());

    std::vector<double> localKData(localDOFs * numPlasticParams_);
    fem->compute_d2E_dxda(data->elementCacheData[ele].get(), localKData.data());

    ES::Mp<ES::MXd> localK(localKData.data(), localDOFs, numPlasticParams_);
    localK *= elementWeights[ele];

    const auto &idxM = element_dfda_InverseIndices[ele];

    for (int localRow = 0; localRow < localDOFs; localRow++) {
      for (int va = 0; va < numPlasticParams_; va++) {
        std::ptrdiff_t offset = idxM(localRow, va);
        if (offset >= 0) {
          std::atomic_ref<double> hessRef(hess.valuePtr()[offset]);
          hessRef.fetch_add(localK(localRow, va));
        }
      }
    }
  };

  for (int ele = 0; ele < nele; ele++) {
    localHessFunc(ele);
  }

  if (enableSanityCheck) {
    for (Eigen::Index i = 0; i < hess.nonZeros(); i++) {
      int fpclass = std::fpclassify(hess.valuePtr()[i]);
      if (fpclass == FP_INFINITE || fpclass == FP_NAN) {
        SPDLOG_LOGGER_ERROR(Logging::lgr(), "Encounter weird numbers at {}: {}", i, hess.valuePtr()[i]);
        throw std::logic_error("Encounter weird numbers.");
      }
      else if (fpclass == FP_SUBNORMAL) {
        hess.valuePtr()[i] = 0;
      }
    }
  }
}

void DeformationModelAssembler::compute_df_db(const double *x, EigenSupport::SpMatD &hess) const
{
  if (numElasticParams_ == 0)
    return;

  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  auto localHessFunc = [this, x, &hess](int ele) {
    if (elementWeights[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    dofLayout->gather(ele, x, localp.data());

    const DeformationModel *fem = femModels[ele];
    fem->prepareData(localp.data(), data->elementCacheData[ele].get());

    std::vector<double> localKData(localDOFs * numElasticParams_);
    fem->compute_d2E_dxdb(data->elementCacheData[ele].get(), localKData.data());

    ES::Mp<ES::MXd> localK(localKData.data(), localDOFs, numElasticParams_);
    localK *= elementWeights[ele];

    const auto &idxM = element_dfdb_InverseIndices[ele];

    for (int localRow = 0; localRow < localDOFs; localRow++) {
      for (int va = 0; va < numElasticParams_; va++) {
        std::ptrdiff_t offset = idxM(localRow, va);
        if (offset >= 0) {
          std::atomic_ref<double> hessRef(hess.valuePtr()[offset]);
          hessRef.fetch_add(localK(localRow, va));
        }
      }
    }
  };

  for (int ele = 0; ele < nele; ele++) {
    localHessFunc(ele);
  }

  if (enableSanityCheck) {
    for (Eigen::Index i = 0; i < hess.nonZeros(); i++) {
      int fpclass = std::fpclassify(hess.valuePtr()[i]);
      if (fpclass == FP_INFINITE || fpclass == FP_NAN) {
        SPDLOG_LOGGER_ERROR(Logging::lgr(), "Encounter weird numbers at {}: {}", i, hess.valuePtr()[i]);
        throw std::logic_error("Encounter weird numbers.");
      }
      else if (fpclass == FP_SUBNORMAL) {
        hess.valuePtr()[i] = 0;
      }
    }
  }
}

void DeformationModelAssembler::computeVonMisesStresses(const double *x, double *elementStresses) const
{
  std::fill(elementStresses, elementStresses + nele, 0.0);

  auto localStressFunc = [this, x, elementStresses](int ele) {
    if (elementWeights[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    dofLayout->gather(ele, x, localp.data());

    const DeformationModel *fem = femModels[ele];
    fem->prepareData(localp.data(), data->elementCacheData[ele].get());

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
    dofLayout->gather(ele, x, localp.data());

    const DeformationModel *fem = femModels[ele];
    fem->prepareData(localp.data(), data->elementCacheData[ele].get());

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
