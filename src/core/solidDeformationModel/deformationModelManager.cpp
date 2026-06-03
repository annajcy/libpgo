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

#include "plasticModel.h"

#include "factories/elasticModelFactory.h"
#include "factories/plasticModelFactory.h"

#include "formulations/elements/parameterizedMaterialBlock.h"

#include "pgoLogging.h"
#include "EigenSupport.h"

#include <fmt/format.h>

#include <tbb/parallel_for.h>
#include <tbb/enumerable_thread_specific.h>

#include <memory>
#include <string>

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

  // elastic models (owned; element models hold non-owning raw pointers)
  std::vector<std::unique_ptr<ElasticModel>> ownedElasticModels;
  // plastic models (owned; element models hold non-owning raw pointers)
  std::vector<std::unique_ptr<PlasticModel>> ownedPlasticModels;

  ES::VXd fiberDirections;
  ES::VXd vertexFiberDirections;
  ES::M3Xd fiberAxesRest, vertexFiberAxesRest;
  ES::M3Xd fiberAxes, vertexFiberAxes;
  ES::M3d globalRotation;

  int numPlasticParams;
  int nele;
  int nvtx;

  // Parameter field ownership.
  std::shared_ptr<OptimizableField> elasticField;
  std::shared_ptr<OptimizableField> plasticField;

  void computeFiberAxes();
};

DeformationModelManagerImpl::~DeformationModelManagerImpl()
{
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
  if (formulation.compatibleMeshType() != meshType)
    throw std::invalid_argument("formulation does not match mesh type");
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

  if (layout->numGlobalDofs() != expectedChannels * expectedElements)
    throw std::invalid_argument(std::string(name) + " global DOF count does not match the mesh.");
}

DeformationModelElasticMaterial inferElasticMaterial(const std::shared_ptr<OptimizableField> &field)
{
  if (!field)
    throw std::invalid_argument("elasticField must be non-null.");
  return ElasticModelFactory::materialFromModelId(field->spec().modelId);
}

DeformationModelPlasticMaterial inferPlasticMaterial(const std::shared_ptr<OptimizableField> &field)
{
  if (!field)
    throw std::invalid_argument("plasticField must be non-null.");
  return PlasticModelFactory::materialFromModelId(field->spec().modelId);
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
  const Formulation &formulation,
  std::shared_ptr<OptimizableField> elasticField,
  std::shared_ptr<OptimizableField> plasticField,
  int enforceSPD, const double *elementFiberDirections, const double *vertexFiberDirections)
{
  initBase(simulationMesh, elementFiberDirections, vertexFiberDirections);
  validateFormulation(simulationMesh.getElementType(), formulation);

  const auto plasticModelType = inferPlasticMaterial(plasticField);
  const auto elasticMaterialType = inferElasticMaterial(elasticField);
  const auto *mat = simulationMesh.getElementMaterial(0, 0);
  const int ne = mat->numElasticParameters(elasticMaterialType);
  const int np = mat->numPlasticParameters(plasticModelType);
  data->numPlasticParams = np;

  validateParameterField("elasticField", elasticField.get(), ParameterDomain::ELASTIC, ne, data->nele);
  validateParameterField("plasticField", plasticField.get(), ParameterDomain::PLASTIC, np, data->nele);

  data->elasticField = std::move(elasticField);
  data->plasticField = std::move(plasticField);

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

  data->elementFEMs.resize(nele);
  data->ownedElasticModels.resize(nele);
  data->ownedPlasticModels.resize(nele);

  // Per-element FEM creation (all elements in parallel).
  tbb::parallel_for(
    0, nele, [&](int ele) {
      const double *fiberDir = nullptr;
      if (data->fiberAxesRest.size() > 0) {
        fiberDir = data->fiberAxesRest.block<3, 3>(0, ele * 3).row(0).data();
      }

      data->ownedElasticModels[ele] = ElasticModelFactory::create(
        *data->simulationMesh, ele, elasticMaterialType, fiberDir);

      const double *fiberAxesRest = (data->fiberAxesRest.size() > 0)
        ? data->fiberAxesRest.data() + ele * 9 : nullptr;

      data->ownedPlasticModels[ele] = PlasticModelFactory::create(plasticModelType, fiberAxesRest);

      ElasticBlock elasticBlock{data->ownedElasticModels[ele].get(), data->elasticField.get()};
      PlasticBlock plasticBlock{data->ownedPlasticModels[ele].get(), data->plasticField.get()};

      data->elementFEMs[ele] = ElementModelFactory::create(
        *data->simulationMesh, ele, elasticBlock, plasticBlock, formulation);
    },
    tbb::static_partitioner());

  // Parameter values are owned by the explicit fields passed to the manager.
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

  for (const auto &mat : data->ownedElasticModels) {
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

  if (!data->ownedPlasticModels.empty() && data->fiberAxes.cols() > 0) {
    tbb::parallel_for(
      0, data->nele, [this](int ele) {
        data->ownedPlasticModels[ele]->setFiberAxes(data->fiberAxes.data() + ele * 9);
      },
      tbb::static_partitioner());
  }
}

void DeformationModelManager::getVertexAlignedMatrix(int id, double R[9]) const
{
  (Eigen::Map<ES::M3d>(R)) = data->vertexFiberAxes.block<3, 3>(0, id * 3);
}

void DeformationModelManager::getElementAlignedMatrix(int id, double R[9]) const
{
  bool isIdentity = data->ownedPlasticModels[id]->isIdentityTransform();
  if (isIdentity || data->fiberAxes.cols() < (id + 1) * 3) {
    (Eigen::Map<ES::M3d>(R)) = ES::M3d::Identity();
  }
  else {
    (Eigen::Map<ES::M3d>(R)) = data->fiberAxes.block<3, 3>(0, id * 3);
  }
}

void DeformationModelManager::setElementAlignedMatrix(int id, double R[9])
{
  if (data->fiberAxesRest.cols() < (id + 1) * 3)
    return;

  data->fiberAxesRest.block<3, 3>(0, id * 3) = Eigen::Map<ES::M3d>(R);
  data->fiberAxes.block<3, 3>(0, id * 3) = data->fiberAxesRest.block<3, 3>(0, id * 3) * data->globalRotation.transpose();

  data->ownedPlasticModels[id]->setFiberAxes(data->fiberAxes.data() + id * 9);
}

int DeformationModelManager::getNumPlasticParameters() const
{
  return data->numPlasticParams;
}

int DeformationModelManager::getNumElasticParameters() const
{
  return data->ownedElasticModels[0]->getNumParameters();
}

ES::VXd DeformationModelManager::getElasticParameterSnapshot() const
{
  const auto *layout = data->elasticField->dofLayout();
  const int n = layout ? layout->numGlobalDofs() : 0;
  ES::VXd params(n);
  if (n > 0)
    params = Eigen::Map<const ES::VXd>(data->elasticField->globalData(), n);
  return params;
}

ES::VXd DeformationModelManager::getPlasticParameterSnapshot() const
{
  const auto *layout = data->plasticField->dofLayout();
  const int n = layout ? layout->numGlobalDofs() : 0;
  ES::VXd params(n);
  if (n > 0)
    params = Eigen::Map<const ES::VXd>(data->plasticField->globalData(), n);
  return params;
}
