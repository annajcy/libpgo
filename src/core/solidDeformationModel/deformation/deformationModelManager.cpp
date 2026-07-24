/*
author: Bohan Wang
copyright to USC, MIT, NUS
*/

#include "deformation/deformationModelManager.h"

#include "deformation/deformationModel.h"
#include "formulations/formulation/formulation.h"
#include "simulation/simulationMesh.h"
#include "material/elastic/elasticModel.h"
#include "material/plastic/plasticModel.h"
#include "pgoLogging.h"

#include <stdexcept>
#include <string>
#include <tbb/parallel_for.h>
#include <vector>

namespace pgo::SolidDeformationModel
{

class DeformationModelManagerImpl
{
public:
  std::shared_ptr<const SimulationMesh> mesh;
  std::shared_ptr<const MaterialFrameField> materialFrames;
  std::shared_ptr<const ElasticModelConfig> elasticConfig;
  std::shared_ptr<const PlasticModelConfig> plasticConfig;
  std::vector<std::unique_ptr<DeformationModel>> elementFEMs;
  int numPlasticParams = 0;
  int nele = 0;
};

namespace
{
void validateFormulation(
  SimulationMeshType meshType, const Formulation &formulation)
{
  if (formulation.compatibleMeshType() != meshType)
    throw std::invalid_argument("formulation does not match mesh type");
}

void validateFrameRequirement(
  const MaterialFrameField &field,
  MaterialFrameRequirement requirement,
  const char *modelDomain)
{
  if (!field.satisfies(requirement))
    throw std::invalid_argument(
      std::string(modelDomain) +
      " material requires a material frame capability that the field does not provide.");
}
}  // namespace

DeformationModelManager::DeformationModelManager(
  std::shared_ptr<const SimulationMesh> mesh,
  std::shared_ptr<const ElasticModelConfig> elasticConfig,
  std::shared_ptr<const PlasticModelConfig> plasticConfig,
  const Formulation &formulation,
  int enforceSPD):
  DeformationModelManager(
    mesh,
    std::move(elasticConfig),
    std::move(plasticConfig),
    formulation,
    enforceSPD,
    makeGlobalAxesMaterialFrameField(
      mesh ? mesh->getNumElements() : 0))
{
}

DeformationModelManager::DeformationModelManager(
  std::shared_ptr<const SimulationMesh> mesh,
  std::shared_ptr<const ElasticModelConfig> elasticConfig,
  std::shared_ptr<const PlasticModelConfig> plasticConfig,
  const Formulation &formulation,
  int enforceSPD,
  std::shared_ptr<const MaterialFrameField> materialFrames)
{
  if (!mesh)
    throw std::invalid_argument(
      "DeformationModelManager: mesh must be non-null.");
  if (!materialFrames)
    throw std::invalid_argument(
      "DeformationModelManager: materialFrames must be non-null.");
  if (!elasticConfig || !plasticConfig)
    throw std::invalid_argument("DeformationModelManager: model configs must be non-null.");
  if (mesh->getNumElements() <= 0)
    throw std::invalid_argument(
      "DeformationModelManager: mesh must contain at least one element.");
  if (materialFrames->numElements() != mesh->getNumElements())
    throw std::invalid_argument(
      "DeformationModelManager: material frame element count does not match the mesh.");

  data = std::make_unique<DeformationModelManagerImpl>();
  data->mesh = std::move(mesh);
  data->materialFrames = std::move(materialFrames);
  data->elasticConfig = std::move(elasticConfig);
  data->plasticConfig = std::move(plasticConfig);
  data->nele = data->mesh->getNumElements();

  validateFormulation(data->mesh->getElementType(), formulation);
  validateFrameRequirement(
    *data->materialFrames,
    data->elasticConfig->frameRequirement(),
    "elastic");
  validateFrameRequirement(
    *data->materialFrames,
    data->plasticConfig->frameRequirement(),
    "plastic");

  initImpl(formulation);

  data->numPlasticParams =
    data->elementFEMs[0]->getNumPlasticParameters();
  if (enforceSPD)
    setEnforceSPD(enforceSPD);
}

void DeformationModelManager::initImpl(const Formulation &formulation)
{
  SPDLOG_LOGGER_INFO(
    pgo::Logging::lgr(), "Initializing element models (manager path)...");

  data->elementFEMs.resize(data->nele);
  tbb::parallel_for(0, data->nele, [&](int ele) {
    const MaterialFrame materialToReference =
      data->materialFrames->materialToReferenceFrame(ele, 0);
    auto em = data->elasticConfig->createModel(
      *data->mesh, ele, materialToReference);
    auto pm = data->plasticConfig->createModel(
      *data->mesh, ele, materialToReference);
    data->elementFEMs[ele] = formulation.createElement(
      *data->mesh, ele, std::move(em), std::move(pm));
  });
}

DeformationModelManager::~DeformationModelManager() = default;

void DeformationModelManager::setEnforceSPD(int enable)
{
  for (const auto &model : data->elementFEMs) {
    if (model)
      model->enableSPD(enable);
  }
}

int DeformationModelManager::getNumPlasticParameters() const
{
  return data->numPlasticParams;
}

int DeformationModelManager::getNumElasticParameters() const
{
  return data->elementFEMs[0]->getNumElasticParameters();
}

const SimulationMesh *DeformationModelManager::getMesh() const
{
  return data->mesh.get();
}

const MaterialFrameField &
DeformationModelManager::materialFrameField() const
{
  return *data->materialFrames;
}

std::shared_ptr<const MaterialFrameField>
DeformationModelManager::materialFrameFieldPtr() const
{
  return data->materialFrames;
}

std::shared_ptr<const ElasticModelConfig>
DeformationModelManager::elasticModelConfig() const
{
  return data->elasticConfig;
}

std::shared_ptr<const PlasticModelConfig>
DeformationModelManager::plasticModelConfig() const
{
  return data->plasticConfig;
}

MaterialFrame DeformationModelManager::materialToReferenceFrame(
  int elementId, int quadratureId) const
{
  return data->materialFrames->materialToReferenceFrame(
    elementId, quadratureId);
}

const DeformationModel *
DeformationModelManager::getDeformationModel(int eleID) const
{
  if (eleID < 0 || eleID >= data->nele)
    throw std::out_of_range(
      "DeformationModelManager: element index is out of range.");
  return data->elementFEMs[eleID].get();
}

}  // namespace pgo::SolidDeformationModel
