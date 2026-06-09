/*
author: Bohan Wang
copyright to USC, MIT, NUS
*/

#include "deformation/deformationModelManager.h"

#include "deformation/deformationModel.h"
#include "formulations/formulation/formulation.h"

#include "simulation/simulationMesh.h"

#include "material/elastic/elasticModel.h"
#include "material/elastic/elasticModel3DDeformationGradient.h"

#include "material/plastic/plasticModel.h"

#include "material/elastic/elasticModelFactory.h"
#include "material/plastic/plasticModelFactory.h"

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

  std::shared_ptr<const SimulationMesh> mesh;
  DeformationModelElasticMaterial elasticMaterial;
  DeformationModelPlasticMaterial plasticMaterial;
  std::vector<std::unique_ptr<DeformationModel>> elementFEMs;

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
}

void DeformationModelManagerImpl::computeFiberAxes()
{
  SPDLOG_LOGGER_INFO(pgo::Logging::lgr(), "Computing the fiber alignment transformation...");

  std::vector<std::vector<int>> vertexNearbyElements(nvtx);
  std::vector<std::vector<int>> elementNearbyElements(nele);

  for (int i = 0; i < nele; i++) {
    for (int j = 0; j < mesh->getNumElementVertices(); j++) {
      vertexNearbyElements[mesh->getVertexIndex(i, j)].push_back(i);
    }
  }

  for (int i = 0; i < nele; i++) {
    for (int j = 0; j < mesh->getNumElementVertices(); j++) {
      const auto &eles = vertexNearbyElements[mesh->getVertexIndex(i, j)];
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

}  // namespace

void DeformationModelManager::initFiber(
  const double *elementFiberDirections, const double *vertexFiberDirections)
{
  data->nele = data->mesh->getNumElements();
  data->nvtx = data->mesh->getNumVertices();

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

DeformationModelManager::DeformationModelManager(std::shared_ptr<const SimulationMesh> mesh,
  DeformationModelElasticMaterial elasticMaterial,
  DeformationModelPlasticMaterial plasticModelType,
  const Formulation &formulation,
  int enforceSPD, const double *elementFiberDirections, const double *vertexFiberDirections)
{
  if (!mesh)
    throw std::invalid_argument("DeformationModelManager: mesh must be non-null.");
  data = std::make_unique<DeformationModelManagerImpl>();
  data->mesh = std::move(mesh);
  data->elasticMaterial = elasticMaterial;
  data->plasticMaterial = plasticModelType;
  const SimulationMesh &simulationMesh = *data->mesh;
  initFiber(elementFiberDirections, vertexFiberDirections);
  validateFormulation(simulationMesh.getElementType(), formulation);

  initImpl(data->plasticMaterial, data->elasticMaterial, formulation);

  // Parameter-field channel counts are the differentiable parameter counts reported
  // by the created models (the single source of truth).
  const int ne = data->elementFEMs[0]->getNumElasticParameters();
  const int np = data->elementFEMs[0]->getNumPlasticParameters();
  data->numPlasticParams = np;
  (void)ne;

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

  // Per-element FEM creation (all elements in parallel).
  tbb::parallel_for(
    0, nele, [&](int ele) {
      const double *fiberDir = nullptr;
      if (data->fiberAxesRest.size() > 0) {
        fiberDir = data->fiberAxesRest.block<3, 3>(0, ele * 3).row(0).data();
      }

      auto em = ElasticModelFactory::create(
        *data->mesh, ele, elasticMaterialType, fiberDir);

      const double *fiberAxesRest = (data->fiberAxesRest.size() > 0)
        ? data->fiberAxesRest.data() + ele * 9 : nullptr;

      auto pm = PlasticModelFactory::create(plasticModelType, fiberAxesRest);

      data->elementFEMs[ele] = formulation.createElement(
        *data->mesh, ele,
        std::move(em), std::move(pm));
    },
    tbb::static_partitioner());

  // Parameter values are owned by the assembler.
}

const DeformationModel *DeformationModelManager::getDeformationModel(int eleID) const
{
  return data->elementFEMs[eleID].get();
}

DeformationModelManager::~DeformationModelManager() = default;

void DeformationModelManager::setEnforceSPD(int enable)
{
  for (const auto &dm : data->elementFEMs) {
    if (dm)
      dm->enableSPD(enable);
  }
}

const SimulationMesh *DeformationModelManager::getMesh() const
{
  return data->mesh.get();
}

void DeformationModelManager::updateMeshRigidTransformation(const double R[9])
{
  data->globalRotation = Eigen::Map<const ES::M3d>(R);
  auto rotateAxes = [this](ES::M3Xd &axes, const ES::M3Xd &axesRest) {
    tbb::parallel_for(0, (int)axes.cols() / 3, [&](int i) {
      axes.block<3, 3>(0, i * 3) = axesRest.block<3, 3>(0, i * 3) * data->globalRotation.transpose();
    }, tbb::static_partitioner());
  };
  rotateAxes(data->fiberAxes, data->fiberAxesRest);
  rotateAxes(data->vertexFiberAxes, data->vertexFiberAxesRest);

  if (!data->elementFEMs.empty() && data->fiberAxes.cols() > 0) {
    tbb::parallel_for(
      0, data->nele, [this](int ele) {
        data->elementFEMs[ele]->setPlasticFiberAxes(data->fiberAxes.data() + ele * 9);
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
  bool isIdentity = data->elementFEMs[id]->isPlasticIdentityTransform();
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

  data->elementFEMs[id]->setPlasticFiberAxes(data->fiberAxes.data() + id * 9);
}

int DeformationModelManager::getNumPlasticParameters() const
{
  return data->numPlasticParams;
}

int DeformationModelManager::getNumElasticParameters() const
{
  return data->elementFEMs[0]->getNumElasticParameters();
}
