/*
author: Bohan Wang
copyright to USC, MIT, NUS
*/

#include "deformationModelManager.h"

#include "deformationModel.h"
#include "factories/elementModelFactory.h"

#include "simulationMesh.h"

#include "elasticModel.h"
#include "elasticModel3DDeformationGradient.h"
#include "elasticModelCombinedMaterial.h"
#include "elasticModelHillTypeMaterial.h"
#include "elasticModelInvariantBasedMaterial.h"
#include "elasticModelStableNeoHookeanMaterial.h"
#include "elasticModelVolumeMaterial.h"
#include "invariantBasedMaterialStVK.h"
#include "elasticModelLinearMaterial.h"
#include "elasticModel3DSTVKMaterial.h"
#include "elasticModel3DMooneyRivlin.h"

#include "elasticModel2DFundamentalForms.h"
#include "elasticModel2DFundamentalFormsFabric.h"
#include "elasticModel2DFundamentalFormsSTVK.h"

#include "plasticModel.h"
#include "plasticModel3DDeformationGradient.h"
#include "plasticModel3D3DOF.h"
#include "plasticModel3D6DOF.h"
#include "plasticModel3DConstant.h"

#include "plasticModel2DFundamentalForms.h"
#include "plasticModel2DFundamentalFormsUniformStretch.h"

#include "factories/elasticModelFactory.h"
#include "factories/plasticModelFactory.h"

#include "pgoLogging.h"
#include "EigenSupport.h"

#include <fmt/format.h>

#include <tbb/parallel_for.h>
#include <tbb/enumerable_thread_specific.h>

#include <memory>

namespace ES = pgo::EigenSupport;

namespace pgo
{
namespace SolidDeformationModel
{
class DeformationModelManagerImpl
{
public:
  ~DeformationModelManagerImpl();

  const SimulationMesh *simulationMesh = nullptr;   // non-owning immutable borrow

  std::vector<DeformationModel *> elementFEMs;

  // element elastic material
  // volumetric elastic material
  std::vector<ElasticModelStableNeoHookeanMaterial *> stableNeoHookeanMaterials;
  std::vector<ElasticModelLinearMaterial *> linearMaterials;
  std::vector<ElasticModelHillTypeMaterial *> hillTypeMaterials;
  std::vector<ElasticModelInvariantBasedMaterial *> invariantBasedMaterials;
  std::vector<ElasticModelVolumeMaterial *> volumeMaterials;
  std::vector<ElasticModel3DSTVKMaterial *> stvkMaterials;
  std::vector<ElasticModel3DMooneyRivlin *> mooneyRivlinMaterials;
  std::vector<ElasticModelCombinedMaterial<2> *> combined2Materials;
  std::vector<ElasticModelCombinedMaterial<3> *> combined3Materials;

  // shell elastic material
  std::vector<ElasticModel2DFundamentalFormsFabric *> shellFabricMaterials;
  std::vector<ElasticModel2DFundamentalFormsSTVK *> shellSTVKMaterials;

  std::vector<ElasticModel *> elementMaterials;
  std::vector<InvariantBasedMaterial *> invariantModels;

  // plastic models
  std::vector<PlasticModel3DConstant *> plasticVolConstant;
  std::vector<PlasticModel3D3DOF *> plasticVol3DOF;
  std::vector<PlasticModel3D6DOF *> plasticVol6DOF;

  // plastic model shell
  std::vector<PlasticModel2DFundamentalForms *> plasticShellConstant;
  std::vector<PlasticModel2DFundamentalFormsUniformStretch *> plasticShellUniformStretch;

  ES::VXd fiberDirections;
  ES::VXd vertexFiberDirections;
  ES::M3Xd fiberAxesRest, vertexFiberAxesRest;
  ES::M3Xd fiberAxes, vertexFiberAxes;
  ES::M3d globalRotation;

  int numPlasticParams;
  int nele;
  int nvtx;

  void computeFiberAxes();
};

DeformationModelManagerImpl::~DeformationModelManagerImpl()
{
  for (auto ptr : elementFEMs)
    delete ptr;

  for (auto ptr : stableNeoHookeanMaterials)
    if (ptr)
      delete ptr;

  for (auto ptr : linearMaterials)
    if (ptr)
      delete ptr;

  for (auto ptr : hillTypeMaterials)
    if (ptr)
      delete ptr;

  for (auto ptr : invariantBasedMaterials)
    if (ptr)
      delete ptr;

  for (auto ptr : volumeMaterials)
    if (ptr)
      delete ptr;

  for (auto ptr : combined2Materials)
    if (ptr)
      delete ptr;

  for (auto ptr : combined3Materials)
    if (ptr)
      delete ptr;

  for (auto ptr : stvkMaterials)
    if (ptr)
      delete ptr;

  for (auto ptr : mooneyRivlinMaterials)
    if (ptr)
      delete ptr;

  for (auto ptr : invariantModels)
    if (ptr)
      delete ptr;

  for (auto ptr : shellFabricMaterials)
    if (ptr)
      delete ptr;

  for (auto ptr : shellSTVKMaterials)
    if (ptr)
      delete ptr;

  for (auto ptr : plasticVol3DOF)
    if (ptr)
      delete ptr;

  for (auto ptr : plasticVol6DOF)
    if (ptr)
      delete ptr;

  for (auto ptr : plasticVolConstant)
    if (ptr)
      delete ptr;

  for (auto ptr : plasticShellConstant)
    if (ptr)
      delete ptr;

  for (auto ptr : plasticShellUniformStretch)
    if (ptr)
      delete ptr;
}

void DeformationModelManagerImpl::computeFiberAxes()
{
  SPDLOG_LOGGER_INFO(pgo::Logging::lgr(), "Computing the fiber alignment transformation...");

  std::vector<std::vector<int>> vertexNearbyElements(nvtx);
  std::vector<std::vector<int>> elementNearbyElements(nele);

  for (int i = 0; i < nele; i++) {
    for (int j = 0; j < simulationMesh->getNumElementVertices(); j++) {
      vertexNearbyElements[simulationMesh->getVertexIndex(i, j)].push_back(i);
    }
  }

  for (int i = 0; i < nele; i++) {
    for (int j = 0; j < simulationMesh->getNumElementVertices(); j++) {
      const auto &eles = vertexNearbyElements[simulationMesh->getVertexIndex(i, j)];
      elementNearbyElements[i].insert(elementNearbyElements[i].end(), eles.begin(), eles.end());
    }

    // sort and remove duplications
    std::sort(elementNearbyElements[i].begin(), elementNearbyElements[i].end());
    auto itt = std::unique(elementNearbyElements[i].begin(), elementNearbyElements[i].end());
    elementNearbyElements[i].erase(itt, elementNearbyElements[i].end());

    // remove itself
    itt = std::lower_bound(elementNearbyElements[i].begin(), elementNearbyElements[i].end(), i);
    PGO_ALOG(itt != elementNearbyElements[i].end() && *itt == i);
    elementNearbyElements[i].erase(itt);
  }

  fiberAxesRest.resize(3, nele * 3);

  ES::V3d guideVector(0, 0, 1);
  std::vector<int> axisComputed(nele, 0);
  while (1) {
    int allComputed = 1;
    for (int f : axisComputed)
      allComputed &= f;

    if (allComputed)
      break;

    for (int ele = 0; ele < nele; ele++) {
      if (axisComputed[ele] == 1)
        continue;

      ES::V3d hintY = guideVector;
      ES::V3d X = fiberDirections.segment<3>(ele * 3);
      X.normalize();

      // if it is the degenerated case
      if (std::abs(X.dot(hintY)) > 1 - 1e-6) {
        // we first search nearby elements
        bool nearbyFound = false;
        for (int elej : elementNearbyElements[ele]) {
          if (axisComputed[elej] == 0) {
            continue;
          }

          // fetch the computed direction
          ES::M3d R = fiberAxesRest.block<3, 3>(0, elej * 3);
          hintY = R.row(1);
          nearbyFound = true;
          break;
        }

        if (nearbyFound == false) {
          continue;
        }
      }

      ES::V3d Z = X.cross(hintY);
      Z.normalize();

      ES::V3d Y = Z.cross(X);
      Y.normalize();

      ES::M3d R;
      R.row(0) = X;
      R.row(1) = Y;
      R.row(2) = Z;

      fiberAxesRest.block<3, 3>(0, ele * 3) = R;
      axisComputed[ele] = 1;
    }

    int numAxes = std::count_if(axisComputed.begin(), axisComputed.end(), [](int val) { return val == 1; });
    SPDLOG_LOGGER_INFO(pgo::Logging::lgr(), "# axes computed:{}", numAxes);
  }

  vertexFiberAxesRest.resize(3, nvtx * 3);
  for (int vi = 0; vi < nvtx; vi++) {
    ES::V3d hintY = fiberAxesRest.col(vertexNearbyElements[vi][0] * 3 + 1);
    ES::V3d X = vertexFiberDirections.segment<3>(vi * 3);
    X.normalize();

    ES::V3d Z = X.cross(hintY);
    Z.normalize();

    ES::V3d Y = Z.cross(X);
    Y.normalize();

    ES::M3d R;
    R.row(0) = X;
    R.row(1) = Y;
    R.row(2) = Z;

    vertexFiberAxesRest.block<3, 3>(0, vi * 3) = R;
  }

  fiberAxes = fiberAxesRest;
  vertexFiberAxes = vertexFiberAxesRest;
}

std::map<DeformationModelPlasticMaterial, int> numPlasticDOFs{
  { DeformationModelPlasticMaterial::VOLUMETRIC_DOF0, 0 },
  { DeformationModelPlasticMaterial::VOLUMETRIC_DOF3, 3 },
  { DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, 6 },

  { DeformationModelPlasticMaterial::SHELL_FF_DOF0, 0 },
  { DeformationModelPlasticMaterial::SHELL_FF_DOF1, 1 },

};

}  // namespace SolidDeformationModel
}  // namespace pgo

using namespace pgo::SolidDeformationModel;

DeformationModelManager::DeformationModelManager(const SimulationMesh &simulationMesh,
  DeformationModelPlasticMaterial plasticModelType, DeformationModelElasticMaterial elasticMaterialType,
  int enforceSPD, const double *elementFiberDirections, const double *vertexFiberDirections)
{
  data = new DeformationModelManagerImpl;

  data->simulationMesh = &simulationMesh;
  data->nele = data->simulationMesh->getNumElements();
  data->nvtx = data->simulationMesh->getNumVertices();

  if (elementFiberDirections)
    data->fiberDirections = Eigen::Map<const ES::VXd>(elementFiberDirections, data->nele * 3);
  else
    data->fiberDirections.setZero(0);

  if (vertexFiberDirections)
    data->vertexFiberDirections = Eigen::Map<const ES::VXd>(vertexFiberDirections, data->nvtx * 3);
  else
    data->vertexFiberDirections.setZero(0);

  initImpl(plasticModelType, elasticMaterialType);

  if (enforceSPD)
    setEnforceSPD(enforceSPD);
}

DeformationModelManager::~DeformationModelManager()
{
  delete data;
}





void DeformationModelManager::initImpl(DeformationModelPlasticMaterial plasticModelType, DeformationModelElasticMaterial elasticMaterialType)
{
  SPDLOG_LOGGER_INFO(pgo::Logging::lgr(), "Initializing element models (manager path)...");

  if (data->fiberDirections.size() || data->vertexFiberDirections.size()) {
    data->computeFiberAxes();
  }

  auto it = numPlasticDOFs.find(plasticModelType);
  PGO_ALOG(it != numPlasticDOFs.end());
  data->numPlasticParams = it->second;
  data->globalRotation = ES::M3d::Identity();

  // Allocate storage vectors (ownership tracking, same layout as before).
  if (elasticMaterialType == DeformationModelElasticMaterial::HILL_STABLE_NEO ||
    elasticMaterialType == DeformationModelElasticMaterial::HILL_STVK ||
    elasticMaterialType == DeformationModelElasticMaterial::HILL_STVK_VOL) {
    data->hillTypeMaterials.assign(data->nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::HILL_STABLE_NEO ||
    elasticMaterialType == DeformationModelElasticMaterial::STABLE_NEO) {
    data->stableNeoHookeanMaterials.assign(data->nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::HILL_STVK ||
    elasticMaterialType == DeformationModelElasticMaterial::HILL_STVK_VOL ||
    elasticMaterialType == DeformationModelElasticMaterial::INV_STVK ||
    elasticMaterialType == DeformationModelElasticMaterial::STVK_VOL) {
    data->invariantModels.assign(data->nele, nullptr);
    data->invariantBasedMaterials.assign(data->nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::HILL_STVK_VOL ||
    elasticMaterialType == DeformationModelElasticMaterial::VOLUME ||
    elasticMaterialType == DeformationModelElasticMaterial::STVK_VOL) {
    data->volumeMaterials.assign(data->nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::HILL_STABLE_NEO ||
    elasticMaterialType == DeformationModelElasticMaterial::HILL_STVK ||
    elasticMaterialType == DeformationModelElasticMaterial::STVK_VOL) {
    data->combined2Materials.assign(data->nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::HILL_STVK_VOL) {
    data->combined3Materials.assign(data->nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::LINEAR) {
    data->linearMaterials.assign(data->nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::STVK) {
    data->stvkMaterials.assign(data->nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::MOONEY_RIVLIN) {
    data->mooneyRivlinMaterials.assign(data->nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::KOITER_FABRIC) {
    data->shellFabricMaterials.assign(data->nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::KOITER_STVK) {
    data->shellSTVKMaterials.assign(data->nele, nullptr);
  }

  data->elementFEMs.assign(data->nele, nullptr);
  data->elementMaterials.assign(data->nele, nullptr);

  if (plasticModelType == DeformationModelPlasticMaterial::VOLUMETRIC_DOF0) {
    data->plasticVolConstant.assign(data->nele, nullptr);
  }
  else if (plasticModelType == DeformationModelPlasticMaterial::VOLUMETRIC_DOF3) {
    data->plasticVol3DOF.assign(data->nele, nullptr);
  }
  else if (plasticModelType == DeformationModelPlasticMaterial::VOLUMETRIC_DOF6) {
    data->plasticVol6DOF.assign(data->nele, nullptr);
  }
  else if (plasticModelType == DeformationModelPlasticMaterial::SHELL_FF_DOF0) {
    data->plasticShellConstant.assign(data->nele, nullptr);
  }
  else if (plasticModelType == DeformationModelPlasticMaterial::SHELL_FF_DOF1) {
    data->plasticShellUniformStretch.assign(data->nele, nullptr);
  }

  tbb::parallel_for(
    0, data->nele, [&](int ele) {
      // Fiber direction (row 0 of fiberAxesRest) for Hill-type materials.
      const double *fiberDir = nullptr;
      if (data->fiberAxesRest.size() > 0) {
        fiberDir = data->fiberAxesRest.block<3, 3>(0, ele * 3).row(0).data();
      }

      // Create elastic model via factory.
      auto elasticResult = ElasticModelFactory::create(
        *data->simulationMesh, ele, elasticMaterialType, fiberDir);

      data->elementMaterials[ele] = elasticResult.elementMaterial;
      if (elasticResult.stableNeo) data->stableNeoHookeanMaterials[ele] = elasticResult.stableNeo;
      if (elasticResult.linear) data->linearMaterials[ele] = elasticResult.linear;
      if (elasticResult.hill) data->hillTypeMaterials[ele] = elasticResult.hill;
      if (elasticResult.invariantBased) data->invariantBasedMaterials[ele] = elasticResult.invariantBased;
      if (elasticResult.volume) data->volumeMaterials[ele] = elasticResult.volume;
      if (elasticResult.stvk) data->stvkMaterials[ele] = elasticResult.stvk;
      if (elasticResult.mooneyRivlin) data->mooneyRivlinMaterials[ele] = elasticResult.mooneyRivlin;
      if (elasticResult.combined2) data->combined2Materials[ele] = elasticResult.combined2;
      if (elasticResult.combined3) data->combined3Materials[ele] = elasticResult.combined3;
      if (elasticResult.shellFabric) data->shellFabricMaterials[ele] = elasticResult.shellFabric;
      if (elasticResult.shellSTVK) data->shellSTVKMaterials[ele] = elasticResult.shellSTVK;
      if (elasticResult.invariantModel) data->invariantModels[ele] = elasticResult.invariantModel;

      // Create plastic model via factory.
      const double *fiberAxesRest = (data->fiberAxesRest.size() > 0)
        ? data->fiberAxesRest.data() + ele * 9 : nullptr;

      auto plasticResult = PlasticModelFactory::create(
        *data->simulationMesh, ele, plasticModelType, fiberAxesRest);

      if (plasticResult.volConstant) data->plasticVolConstant[ele] = plasticResult.volConstant;
      if (plasticResult.vol3DOF) data->plasticVol3DOF[ele] = plasticResult.vol3DOF;
      if (plasticResult.vol6DOF) data->plasticVol6DOF[ele] = plasticResult.vol6DOF;
      if (plasticResult.shellConstant) data->plasticShellConstant[ele] = plasticResult.shellConstant;
      if (plasticResult.shellUniformStretch) data->plasticShellUniformStretch[ele] = plasticResult.shellUniformStretch;

      // Create element FEM through ElementModelFactory.
      if (data->simulationMesh->getElementType() == SimulationMeshType::TET) {
        data->elementFEMs[ele] = ElementModelFactory::create<TetP1>(
          *data->simulationMesh, ele, data->elementMaterials[ele],
          plasticResult.model, elasticMaterialType);
      } else if (data->simulationMesh->getElementType() == SimulationMeshType::CUBIC) {
        data->elementFEMs[ele] = ElementModelFactory::create<HexTrilinear>(
          *data->simulationMesh, ele, data->elementMaterials[ele],
          plasticResult.model, elasticMaterialType);
      }
      else if (data->simulationMesh->getElementType() == SimulationMeshType::SHELL) {
        data->elementFEMs[ele] = ElementModelFactory::create<ShellKoiter>(
          *data->simulationMesh, ele, data->elementMaterials[ele],
          plasticResult.model, elasticMaterialType);
      }
      else {
        throw std::logic_error("unknown mesh element type");
      }
    },
    tbb::static_partitioner());
}
const DeformationModel *DeformationModelManager::getDeformationModel(int eleID) const
{
  return data->elementFEMs[eleID];
}

void DeformationModelManager::setEnforceSPD(int enable)
{
  for (auto dm : data->elementFEMs) {
    if (dm)
      dm->enableSPD(enable);
  }

  for (auto mat : data->elementMaterials) {
    if (mat)
      mat->enableSPD(enable);
  }
}

const SimulationMesh *DeformationModelManager::getMesh() const
{
  return data->simulationMesh;
}

void DeformationModelManager::updateMeshRigidTransformation(const double R[9])
{
  data->globalRotation = Eigen::Map<const ES::M3d>(R);
  tbb::parallel_for(
    0, (int)data->fiberAxes.cols() / 3, [this](int i) {
      data->fiberAxes.block<3, 3>(0, i * 3) = data->fiberAxesRest.block<3, 3>(0, i * 3) * data->globalRotation.transpose();
    },
    tbb::static_partitioner());

  tbb::parallel_for(
    0, (int)data->vertexFiberAxes.cols() / 3, [this](int i) {
      data->vertexFiberAxes.block<3, 3>(0, i * 3) = data->vertexFiberAxesRest.block<3, 3>(0, i * 3) * data->globalRotation.transpose();
    },
    tbb::static_partitioner());

  tbb::parallel_for(
    0, data->nele, [this](int ele) {
      if (data->plasticVol3DOF[ele])
        data->plasticVol3DOF[ele]->setR(data->fiberAxes.data() + ele * 9);
    },
    tbb::static_partitioner());
}

void DeformationModelManager::getVertexAlignedMatrix(int id, double R[9]) const
{
  (Eigen::Map<ES::M3d>(R)) = data->vertexFiberAxes.block<3, 3>(0, id * 3);
}

void DeformationModelManager::getElementAlignedMatrix(int id, double R[9]) const
{
  if (data->plasticVol6DOF[id] || data->plasticVolConstant[id]) {
    (Eigen::Map<ES::M3d>(R)) = ES::M3d::Identity();
  }
  else {
    (Eigen::Map<ES::M3d>(R)) = data->fiberAxes.block<3, 3>(0, id * 3);
  }
}

void DeformationModelManager::setElementAlignedMatrix(int id, double R[9])
{
  data->fiberAxesRest.block<3, 3>(0, id * 3) = Eigen::Map<ES::M3d>(R);
  data->fiberAxes.block<3, 3>(0, id * 3) = data->fiberAxesRest.block<3, 3>(0, id * 3) * data->globalRotation.transpose();

  if (data->plasticVol3DOF[id] == nullptr)
    return;

  data->plasticVol3DOF[id]->setR(data->fiberAxes.data() + id * 9);
}

int DeformationModelManager::getNumPlasticParameters() const
{
  return data->numPlasticParams;
}

int DeformationModelManager::getNumElasticParameters() const
{
  return data->elementMaterials[0]->getNumParameters();
}
