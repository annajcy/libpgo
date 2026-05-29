/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "deformationModelAssembler.h"
#include "materialMaxStepPolynomialUtils.h"
#include "deformationModelManager.h"
#include "simulationMesh.h"
#include "deformationModel.h"
#include "elasticModel.h"
#include "plasticModel.h"
#include "formulations/dof/vertex3DofLayout.h"

#include "pgoLogging.h"
#include "EigenSupport.h"
#include "fmtEigen.h"

#include <tbb/parallel_for.h>
#include <tbb/enumerable_thread_specific.h>

#include <algorithm>
#include <atomic>

using namespace pgo::SolidDeformationModel;

namespace ES = pgo::EigenSupport;

namespace pgo::SolidDeformationModel
{
class DeformationModelAssemblerCacheData
{
public:
  tbb::enumerable_thread_specific<double> energyLocalBuffer;
  tbb::enumerable_thread_specific<ES::MXd> upperRightBlockTLS, lowerRightBlockTLS;
  tbb::enumerable_thread_specific<ES::VXd> gradBlockTLS;

  tbb::affinity_partitioner partitioners[5];
  std::vector<std::unique_ptr<DeformationModel::CacheData>> elementCacheData;
};
}  // namespace pgo::SolidDeformationModel

namespace
{
const double *paramPtr(const ES::VXd &param)
{
  return param.size() ? param.data() : nullptr;
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

void warnIllegalInitialState(pgo::SolidDeformationModel::SimulationMeshType meshType, int elementId, int locationId, double phi0, double eps)
{
  if (locationId >= 0) {
    SPDLOG_LOGGER_WARN(pgo::Logging::lgr(),
      "Phase 1.5 material max step encountered illegal initial state on meshType={} element={} location={} : phi(0)={} <= eps={}. Returning recovery clamp {}.",
      meshTypeName(meshType), elementId, locationId, phi0, eps, pgo::SolidDeformationModel::kMaterialMaxStepMinClamp);
  }
  else {
    SPDLOG_LOGGER_WARN(pgo::Logging::lgr(),
      "Phase 1.5 material max step encountered illegal initial state on meshType={} element={} : phi(0)={} <= eps={}. Returning recovery clamp {}.",
      meshTypeName(meshType), elementId, phi0, eps, pgo::SolidDeformationModel::kMaterialMaxStepMinClamp);
  }
}
}

DeformationModelAssembler::DeformationModelAssembler(std::unique_ptr<const DeformationModelManager> dm, std::unique_ptr<const DofLayout> dof, const double *elementFlags_):
  deformationModelManager(std::move(dm)),
  dofLayout(std::move(dof))
{
  nele = deformationModelManager->getMesh()->getNumElements();
  nvtx = deformationModelManager->getMesh()->getNumVertices();
  neleVtx = deformationModelManager->getMesh()->getNumElementVertices();
  localDOFs = dofLayout->numLocalDofs(0);
  numDOFs = dofLayout->numGlobalDofs();

  numElasticParams = deformationModelManager->getDeformationModel(0)->getElasticModel()->getNumParameters();
  numPlasticParams = deformationModelManager->getDeformationModel(0)->getPlasticModel()->getNumParameters();

  if (elementFlags_) {
    elementFlags.assign(elementFlags_, elementFlags_ + nele);
  }
  else {
    elementFlags.assign(nele, 1);
  }

  restPositions.resize(numDOFs);
  for (int vi = 0; vi < deformationModelManager->getMesh()->getNumVertices(); vi++) {
    ES::V3d p;
    deformationModelManager->getMesh()->getVertex(vi, p.data());
    restPositions.segment<3>(vi * 3) = p;
  }

  data = new DeformationModelAssemblerCacheData;

  for (int i = 0; i < nele; i++) {
    femModels.push_back(deformationModelManager->getDeformationModel(i));
    data->elementCacheData.push_back(femModels.back()->allocateCacheData());
  }

  SPDLOG_LOGGER_INFO(Logging::lgr(), "Assembler parameter:{},{}", numElasticParams, numPlasticParams);

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

  entries.clear();
  for (int ele = 0; ele < nele; ele++) {
    for (int vi = 0; vi < neleVtx; vi++) {
      int vidx = deformationModelManager->getMesh()->getVertexIndex(ele, vi);
      if (vidx < 0) {
        continue;
      }

      for (int dof = 0; dof < 3; dof++) {
        int globalRow = vidx * 3 + dof;
        for (int ep = 0; ep < numElasticParams; ep++)
          entries.emplace_back(globalRow, ele * numElasticParams + ep, 1.0);
      }
    }
  }
  dfdbTemplate.resize(numDOFs, nele * numElasticParams);
  dfdbTemplate.setFromTriplets(entries.begin(), entries.end());

  element_dfdb_InverseIndices.resize(nele);
  for (int ele = 0; ele < nele; ele++) {
    const int *vertexIndices = deformationModelManager->getMesh()->getVertexIndices(ele);
    DynamicIndexMatrix idxM(localDOFs, numElasticParams);
    idxM.setConstant(-1);

    // upper-left block
    for (int vi = 0; vi < neleVtx; vi++) {
      for (int dofi = 0; dofi < 3; dofi++) {
        for (int ep = 0; ep < numElasticParams; ep++) {
          int localRow = vi * 3 + dofi;
          int localCol = ep;

          if (vertexIndices[vi] >= 0) {
            int globalRow = vertexIndices[vi] * 3 + dofi;
            int globalCol = ele * numElasticParams + ep;

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

  entries.clear();
  for (int ele = 0; ele < nele; ele++) {
    for (int vi = 0; vi < neleVtx; vi++) {
      int vidx = deformationModelManager->getMesh()->getVertexIndex(ele, vi);
      if (vidx < 0) {
        continue;
      }
      for (int dof = 0; dof < 3; dof++) {
        int globalRow = vidx * 3 + dof;
        for (int pp = 0; pp < numPlasticParams; pp++)
          entries.emplace_back(globalRow, ele * numPlasticParams + pp, 1.0);
      }
    }
  }
  dfdaTemplate.resize(numDOFs, nele * numPlasticParams);
  dfdaTemplate.setFromTriplets(entries.begin(), entries.end());

  element_dfda_InverseIndices.resize(nele);
  for (int ele = 0; ele < nele; ele++) {
    const int *vertexIndices = deformationModelManager->getMesh()->getVertexIndices(ele);
    DynamicIndexMatrix idxM(localDOFs, numPlasticParams);
    idxM.setConstant(-1);

    // upper-left block
    for (int vi = 0; vi < neleVtx; vi++) {
      for (int dofi = 0; dofi < 3; dofi++) {
        for (int pp = 0; pp < numPlasticParams; pp++) {
          int localRow = vi * 3 + dofi;
          int localCol = pp;

          if (vertexIndices[vi] >= 0) {
            int globalRow = vertexIndices[vi] * 3 + dofi;
            int globalCol = ele * numPlasticParams + pp;

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

DeformationModelAssembler::~DeformationModelAssembler()
{
  if (data)
    delete data;
}

void DeformationModelAssembler::getPlasticParameters(int ele, const double *paramsAll, double *param) const
{
  for (int j = 0; j < numPlasticParams; j++) {
    param[j] = paramsAll[ele * numPlasticParams + j];
  }
}

void DeformationModelAssembler::getElasticParameters(int ele, const double *paramsAll, double *param) const
{
  for (int j = 0; j < numElasticParams; j++) {
    param[j] = paramsAll[ele * numElasticParams + j];
  }
}

double DeformationModelAssembler::computeEnergy(const double *x, const double *plasticParams, const double *elasticParams) const
{
  for (auto it = data->energyLocalBuffer.begin(); it != data->energyLocalBuffer.end(); ++it)
    *it = 0.0;

  auto localEnergyFunc = [this, x, plasticParams, elasticParams](int ele) {
    if (elementFlags[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    dofLayout->gather(ele, x, localp.data());

    ES::VXd plasticParam(numPlasticParams), elasticParam(numElasticParams);
    getPlasticParameters(ele, plasticParams, plasticParam.data());
    getElasticParameters(ele, elasticParams, elasticParam.data());

    const DeformationModel *fem = femModels[ele];
    fem->prepareData(localp.data(), paramPtr(plasticParam), paramPtr(elasticParam), data->elementCacheData[ele].get());
    double energy = fem->computeEnergy(data->elementCacheData[ele].get());

    data->energyLocalBuffer.local() += energy * elementFlags[ele];
  };

  // for (int ele = 0; ele < nele; ele++)
  //   localEnergyFunc(ele);
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
    if (elementFlags[ele] == 0) {
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

void DeformationModelAssembler::computeGradient(const double *x, const double *plasticParams, const double *elasticParams, double *grad) const
{
  memset(grad, 0, sizeof(double) * numDOFs);
  auto localGradFunc = [this, x, plasticParams, elasticParams, grad](int ele) {
    if (elementFlags[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    dofLayout->gather(ele, x, localp.data());

    ES::VXd plasticParam(numPlasticParams), elasticParam(numElasticParams);
    getPlasticParameters(ele, plasticParams, plasticParam.data());
    getElasticParameters(ele, elasticParams, elasticParam.data());

    const DeformationModel *fem = femModels[ele];
    fem->prepareData(localp.data(), paramPtr(plasticParam), paramPtr(elasticParam), data->elementCacheData[ele].get());

    ES::VXd localGradx(localDOFs);
    fem->compute_dE_dx(data->elementCacheData[ele].get(), localGradx.data());
    localGradx *= elementFlags[ele];

    if (enableSanityCheck) {
      for (int i = 0; i < localDOFs; i++) {
        if (std::isfinite(localGradx[i]) == false) {
          SPDLOG_LOGGER_ERROR(Logging::lgr(), "Ele: {}", ele);
          SPDLOG_LOGGER_ERROR(Logging::lgr(), "Encounter weird numbers.\nGrad:\n{}\n;x:{}\n", localGradx, localp);
          SPDLOG_LOGGER_ERROR(Logging::lgr(), "Plastic param: {}\n", plasticParam.transpose());
        }
      }
    }

    dofLayout->scatterAddGradient(ele, localGradx.data(), grad);
  };

  tbb::parallel_for(0, nele, localGradFunc, data->partitioners[1]);

  // clean the numbers
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

void DeformationModelAssembler::computeHessian(const double *x, const double *plasticParams, const double *elasticParams, EigenSupport::SpMatD &hess) const
{
  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  auto localHessFunc = [this, x, plasticParams, elasticParams, &hess](int ele) {
    if (elementFlags[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    dofLayout->gather(ele, x, localp.data());

    ES::VXd plasticParam(numPlasticParams), elasticParam(numElasticParams);
    getPlasticParameters(ele, plasticParams, plasticParam.data());
    getElasticParameters(ele, elasticParams, elasticParam.data());

    const DeformationModel *fem = femModels[ele];
    fem->prepareData(localp.data(), paramPtr(plasticParam), paramPtr(elasticParam), data->elementCacheData[ele].get());

    std::vector<double> localKData(localDOFs * localDOFs);
    fem->compute_d2E_dx2(data->elementCacheData[ele].get(), localKData.data());

    ES::Mp<ES::MXd> localK(localKData.data(), localDOFs, localDOFs);
    localK *= elementFlags[ele];

    const auto &idxM = elementKInverseIndices[ele];

    std::vector<int> globalDofIndices;
    dofLayout->getGlobalDofIndices(ele, globalDofIndices);

    // write matrices in place — vertex-level skip preserves efficiency for shell missing-neighbor slots
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

  // for (int ele = 0; ele < nele; ele++) {
  //   localHessFunc(ele);
  // }

  tbb::parallel_for(0, nele, localHessFunc, data->partitioners[2]);

  // clean the numbers
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

void DeformationModelAssembler::compute_df_da(const double *x, const double *plasticParams, const double *elasticParams, EigenSupport::SpMatD &hess) const
{
  if (numPlasticParams == 0)
    return;

  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  auto localHessFunc = [this, x, plasticParams, elasticParams, &hess](int ele) {
    if (elementFlags[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    dofLayout->gather(ele, x, localp.data());

    ES::VXd plasticParam(numPlasticParams), elasticParam(numElasticParams);
    getPlasticParameters(ele, plasticParams, plasticParam.data());
    getElasticParameters(ele, elasticParams, elasticParam.data());

    const DeformationModel *fem = femModels[ele];
    fem->prepareData(localp.data(), paramPtr(plasticParam), paramPtr(elasticParam), data->elementCacheData[ele].get());

    std::vector<double> localKData(localDOFs * numPlasticParams);
    fem->compute_d2E_dxda(data->elementCacheData[ele].get(), localKData.data());

    ES::Mp<ES::MXd> localK(localKData.data(), localDOFs, numPlasticParams);
    localK *= elementFlags[ele];

    const auto &idxM = element_dfda_InverseIndices[ele];

    // write matrices in place
    for (int localRow = 0; localRow < localDOFs; localRow++) {
      for (int va = 0; va < numPlasticParams; va++) {
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

  // tbb::parallel_for(0, nele, localHessFunc, data->partitioners[2]);

  // clean the numbers
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

void DeformationModelAssembler::compute_df_db(const double *x, const double *plasticParams, const double *elasticParams, EigenSupport::SpMatD &hess) const
{
  if (numElasticParams == 0)
    return;

  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  auto localHessFunc = [this, x, plasticParams, elasticParams, &hess](int ele) {
    if (elementFlags[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    dofLayout->gather(ele, x, localp.data());

    ES::VXd plasticParam(numPlasticParams), elasticParam(numElasticParams);
    getPlasticParameters(ele, plasticParams, plasticParam.data());
    getElasticParameters(ele, elasticParams, elasticParam.data());

    const DeformationModel *fem = femModels[ele];
    fem->prepareData(localp.data(), paramPtr(plasticParam), paramPtr(elasticParam), data->elementCacheData[ele].get());

    std::vector<double> localKData(localDOFs * numElasticParams);
    fem->compute_d2E_dxdb(data->elementCacheData[ele].get(), localKData.data());

    ES::Mp<ES::MXd> localK(localKData.data(), localDOFs, numElasticParams);
    localK *= elementFlags[ele];

    const auto &idxM = element_dfdb_InverseIndices[ele];

    // write matrices in place
    for (int localRow = 0; localRow < localDOFs; localRow++) {
      for (int va = 0; va < numElasticParams; va++) {
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

  // tbb::parallel_for(0, nele, localHessFunc, data->partitioners[2]);

  // clean the numbers
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

void DeformationModelAssembler::computeVonMisesStresses(const double *x, const double *plasticParams, const double *elasticParams, double *elementStresses) const
{
  std::fill(elementStresses, elementStresses + nele, 0.0);

  auto localStressFunc = [this, x, plasticParams, elasticParams, elementStresses](int ele) {
    if (elementFlags[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    dofLayout->gather(ele, x, localp.data());

    ES::VXd plasticParam(numPlasticParams), elasticParam(numElasticParams);
    getPlasticParameters(ele, plasticParams, plasticParam.data());
    getElasticParameters(ele, elasticParams, elasticParam.data());

    const DeformationModel *fem = femModels[ele];
    fem->prepareData(localp.data(), paramPtr(plasticParam), paramPtr(elasticParam), data->elementCacheData[ele].get());

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

void DeformationModelAssembler::computeMaxStrains(const double *x, const double *plasticParams, const double *elasticParams, double *elementStrain) const
{
  std::fill(elementStrain, elementStrain + nele, 0.0);

  auto localStrainFunc = [this, x, plasticParams, elasticParams, elementStrain](int ele) {
    if (elementFlags[ele] == 0)
      return;

    ES::VXd localp(localDOFs);
    dofLayout->gather(ele, x, localp.data());

    ES::VXd plasticParam(numPlasticParams), elasticParam(numElasticParams);
    getPlasticParameters(ele, plasticParams, plasticParam.data());
    getElasticParameters(ele, elasticParams, elasticParam.data());

    const DeformationModel *fem = femModels[ele];
    fem->prepareData(localp.data(), paramPtr(plasticParam), paramPtr(elasticParam), data->elementCacheData[ele].get());

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
