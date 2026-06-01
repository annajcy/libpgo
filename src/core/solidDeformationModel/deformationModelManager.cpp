/*
author: Bohan Wang
copyright to USC, MIT, NUS
*/

#include "deformationModelManager.h"

#include "deformationModel.h"
#include "factories/elementModelFactory.h"
#include "formulations/dof/vertex3DofLayout.h"
#include "formulations/formulation.h"

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

#include "formulations/parameters/constantParameterField.h"
#include "formulations/elements/parameterizedMaterialBlock.h"

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

  std::vector<std::unique_ptr<DeformationModel>> elementFEMs;

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

  // Parameter field ownership.
  std::unique_ptr<ConstantParameterField> elasticField;
  std::unique_ptr<ConstantParameterField> plasticField;
  ES::VXd elasticGlobalParams;
  ES::VXd plasticGlobalParams;

  void computeFiberAxes();
};

DeformationModelManagerImpl::~DeformationModelManagerImpl()
{
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

}  // namespace SolidDeformationModel
}  // namespace pgo

using namespace pgo::SolidDeformationModel;

namespace
{

void validateFormulation(SimulationMeshType meshType, const Formulation &formulation)
{
  switch (meshType) {
  case SimulationMeshType::TET:
    if (!dynamic_cast<const TetFormulation *>(&formulation))
      throw std::invalid_argument("formulation does not match TET mesh");
    return;
  case SimulationMeshType::CUBIC:
    if (!dynamic_cast<const CubicFormulation *>(&formulation))
      throw std::invalid_argument("formulation does not match CUBIC mesh");
    return;
  case SimulationMeshType::SHELL:
    if (!dynamic_cast<const ShellFormulation *>(&formulation))
      throw std::invalid_argument("formulation does not match SHELL mesh");
    return;
  default:
    throw std::logic_error("unsupported mesh element type");
  }
}

}  // namespace

void DeformationModelManager::initBase(const SimulationMesh &simulationMesh,
  const double *elementFiberDirections, const double *vertexFiberDirections)
{
  data = std::make_unique<DeformationModelManagerImpl>();

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

  if (data->fiberDirections.size() || data->vertexFiberDirections.size())
    data->computeFiberAxes();
}

DeformationModelManager::DeformationModelManager(const SimulationMesh &simulationMesh,
  DeformationModelPlasticMaterial plasticModelType, DeformationModelElasticMaterial elasticMaterialType,
  const Formulation &formulation,
  int enforceSPD, const double *elementFiberDirections, const double *vertexFiberDirections)
{
  initBase(simulationMesh, elementFiberDirections, vertexFiberDirections);
  validateFormulation(simulationMesh.getElementType(), formulation);

  const auto *mat = simulationMesh.getElementMaterial(0, 0);
  const int ne = mat->numElasticParameters(elasticMaterialType);
  const int np = mat->numPlasticParameters(plasticModelType);
  data->numPlasticParams = np;

  data->elasticGlobalParams = ES::VXd::Zero(static_cast<Eigen::Index>(data->nele) * ne);
  data->plasticGlobalParams = ES::VXd::Zero(static_cast<Eigen::Index>(data->nele) * np);

  data->elasticField = std::make_unique<ConstantParameterField>(
    ne, data->nele, data->elasticGlobalParams.data());
  data->plasticField = std::make_unique<ConstantParameterField>(
    np, data->nele, data->plasticGlobalParams.data());

  initImpl(plasticModelType, elasticMaterialType, formulation);

  if (enforceSPD)
    setEnforceSPD(enforceSPD);
}

void DeformationModelManager::initImpl(DeformationModelPlasticMaterial plasticModelType,
  DeformationModelElasticMaterial elasticMaterialType,
  const Formulation &formulation)
{
  SPDLOG_LOGGER_INFO(pgo::Logging::lgr(), "Initializing element models (manager path)...");

  data->globalRotation = ES::M3d::Identity();
  const int nele = data->nele;

  // Allocate storage vectors before creating element models.
  if (elasticMaterialType == DeformationModelElasticMaterial::HILL_STABLE_NEO ||
    elasticMaterialType == DeformationModelElasticMaterial::HILL_STVK ||
    elasticMaterialType == DeformationModelElasticMaterial::HILL_STVK_VOL) {
    data->hillTypeMaterials.assign(nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::HILL_STABLE_NEO ||
    elasticMaterialType == DeformationModelElasticMaterial::STABLE_NEO) {
    data->stableNeoHookeanMaterials.assign(nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::HILL_STVK ||
    elasticMaterialType == DeformationModelElasticMaterial::HILL_STVK_VOL ||
    elasticMaterialType == DeformationModelElasticMaterial::INV_STVK ||
    elasticMaterialType == DeformationModelElasticMaterial::STVK_VOL) {
    data->invariantModels.assign(nele, nullptr);
    data->invariantBasedMaterials.assign(nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::HILL_STVK_VOL ||
    elasticMaterialType == DeformationModelElasticMaterial::VOLUME ||
    elasticMaterialType == DeformationModelElasticMaterial::STVK_VOL) {
    data->volumeMaterials.assign(nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::HILL_STABLE_NEO ||
    elasticMaterialType == DeformationModelElasticMaterial::HILL_STVK ||
    elasticMaterialType == DeformationModelElasticMaterial::STVK_VOL) {
    data->combined2Materials.assign(nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::HILL_STVK_VOL) {
    data->combined3Materials.assign(nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::LINEAR) {
    data->linearMaterials.assign(nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::STVK) {
    data->stvkMaterials.assign(nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::MOONEY_RIVLIN) {
    data->mooneyRivlinMaterials.assign(nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::KOITER_FABRIC) {
    data->shellFabricMaterials.assign(nele, nullptr);
  }

  if (elasticMaterialType == DeformationModelElasticMaterial::KOITER_STVK) {
    data->shellSTVKMaterials.assign(nele, nullptr);
  }

  data->elementFEMs.resize(nele);
  data->elementMaterials.assign(nele, nullptr);

  if (plasticModelType == DeformationModelPlasticMaterial::VOLUMETRIC_DOF0) {
    data->plasticVolConstant.assign(nele, nullptr);
  }
  else if (plasticModelType == DeformationModelPlasticMaterial::VOLUMETRIC_DOF3) {
    data->plasticVol3DOF.assign(nele, nullptr);
  }
  else if (plasticModelType == DeformationModelPlasticMaterial::VOLUMETRIC_DOF6) {
    data->plasticVol6DOF.assign(nele, nullptr);
  }
  else if (plasticModelType == DeformationModelPlasticMaterial::SHELL_FF_DOF0) {
    data->plasticShellConstant.assign(nele, nullptr);
  }
  else if (plasticModelType == DeformationModelPlasticMaterial::SHELL_FF_DOF1) {
    data->plasticShellUniformStretch.assign(nele, nullptr);
  }

  // Per-element FEM creation (all elements in parallel).
  tbb::parallel_for(
    0, nele, [&](int ele) {
      const double *fiberDir = nullptr;
      if (data->fiberAxesRest.size() > 0) {
        fiberDir = data->fiberAxesRest.block<3, 3>(0, ele * 3).row(0).data();
      }

      auto eR = ElasticModelFactory::create(
        *data->simulationMesh, ele, elasticMaterialType, fiberDir);

      data->elementMaterials[ele] = eR.elementMaterial;
      if (eR.stableNeo) data->stableNeoHookeanMaterials[ele] = eR.stableNeo;
      if (eR.linear) data->linearMaterials[ele] = eR.linear;
      if (eR.hill) data->hillTypeMaterials[ele] = eR.hill;
      if (eR.invariantBased) data->invariantBasedMaterials[ele] = eR.invariantBased;
      if (eR.volume) data->volumeMaterials[ele] = eR.volume;
      if (eR.stvk) data->stvkMaterials[ele] = eR.stvk;
      if (eR.mooneyRivlin) data->mooneyRivlinMaterials[ele] = eR.mooneyRivlin;
      if (eR.combined2) data->combined2Materials[ele] = eR.combined2;
      if (eR.combined3) data->combined3Materials[ele] = eR.combined3;
      if (eR.shellFabric) data->shellFabricMaterials[ele] = eR.shellFabric;
      if (eR.shellSTVK) data->shellSTVKMaterials[ele] = eR.shellSTVK;
      if (eR.invariantModel) data->invariantModels[ele] = eR.invariantModel;

      const double *fiberAxesRest = (data->fiberAxesRest.size() > 0)
        ? data->fiberAxesRest.data() + ele * 9 : nullptr;

      auto pR = PlasticModelFactory::create(plasticModelType, fiberAxesRest);

      if (pR.volConstant) data->plasticVolConstant[ele] = pR.volConstant;
      if (pR.vol3DOF) data->plasticVol3DOF[ele] = pR.vol3DOF;
      if (pR.vol6DOF) data->plasticVol6DOF[ele] = pR.vol6DOF;
      if (pR.shellConstant) data->plasticShellConstant[ele] = pR.shellConstant;
      if (pR.shellUniformStretch) data->plasticShellUniformStretch[ele] = pR.shellUniformStretch;

      ElasticBlock elasticBlock{eR.elementMaterial, data->elasticField.get()};
      PlasticBlock plasticBlock{pR.model, data->plasticField.get()};

      data->elementFEMs[ele] = ElementModelFactory::create(
        *data->simulationMesh, ele, elasticBlock, plasticBlock, formulation);
    },
    tbb::static_partitioner());

  // Initialize default plastic / elastic parameters.
  {
    std::vector<PlasticModel *> plasticModels(nele);
    for (int ei = 0; ei < nele; ei++)
      plasticModels[ei] = const_cast<PlasticModel *>(data->elementFEMs[ei]->getPlasticModel());
    setPlasticParams(PlasticModelFactory::initializeDefaultPlasticParams(
      nele, data->numPlasticParams, plasticModels.data()));

    const int numElasticParams = data->elementMaterials[0]->getNumParameters();
    setElasticParams(ElasticModelFactory::initializeDefaultElasticParams(
      *data->simulationMesh, elasticMaterialType, numElasticParams));
  }
}

const DeformationModel *DeformationModelManager::getDeformationModel(int eleID) const
{
  return data->elementFEMs[eleID].get();
}

const ParameterField *DeformationModelManager::getElasticParameterField() const
{
  return data->elasticField.get();
}

const ParameterField *DeformationModelManager::getPlasticParameterField() const
{
  return data->plasticField.get();
}

void DeformationModelManager::setElasticParams(const EigenSupport::VXd &params)
{
  data->elasticGlobalParams = params;
  data->elasticField->setGlobalData(data->elasticGlobalParams.data());
}

void DeformationModelManager::setPlasticParams(const EigenSupport::VXd &params)
{
  data->plasticGlobalParams = params;
  data->plasticField->setGlobalData(data->plasticGlobalParams.data());
}

std::unique_ptr<const DofLayout> DeformationModelManager::createDofLayout() const
{
  // Currently all formulations (tet P1, hex trilinear, shell Koiter) use Vertex3DofLayout.
  // Future formulations (e.g. hex tricubic Hermite) will return a different DofLayout.
  return std::make_unique<Vertex3DofLayout>(*getMesh());
}

DeformationModelManager::~DeformationModelManager() = default;

ES::VXd DeformationModelManager::buildRestPosition() const
{
  auto *mesh = getMesh();
  ES::VXd rest(mesh->getNumVertices() * 3);
  for (int vi = 0; vi < mesh->getNumVertices(); vi++) {
    double p[3];
    mesh->getVertex(vi, p);
    rest.segment<3>(vi * 3) = ES::V3d(p[0], p[1], p[2]);
  }
  return rest;
}

void DeformationModelManager::setEnforceSPD(int enable)
{
  for (const auto &dm : data->elementFEMs) {
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

const ES::VXd &DeformationModelManager::getElasticGlobalParams() const
{
  return data->elasticGlobalParams;
}

const ES::VXd &DeformationModelManager::getPlasticGlobalParams() const
{
  return data->plasticGlobalParams;
}
