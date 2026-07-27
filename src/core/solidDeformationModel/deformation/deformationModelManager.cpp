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
#include <algorithm>
#include <span>
#include <tbb/parallel_for.h>
#include <vector>

namespace pgo::SolidDeformationModel
{

class DeformationModelManagerImpl
{
public:
  std::shared_ptr<const SimulationMesh> mesh;
  std::shared_ptr<const MaterialFrameField> materialFrames;
  std::shared_ptr<const ElasticModelDefinition> elasticDefinition;
  std::shared_ptr<const PlasticModelDefinition> plasticDefinition;
  std::shared_ptr<const FixedParameterField> elasticFixed;
  std::shared_ptr<const FixedParameterField> plasticFixed;
  std::shared_ptr<const MaterialParameterization> parameterization;
  std::shared_ptr<const MaterialParameterData> parameterData;
  std::shared_ptr<const MaterialAssignment> assignment;
  std::vector<std::unique_ptr<DeformationModel>> elementFEMs;
  int numElasticParams = 0;
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
  std::shared_ptr<const MaterialAssignment> assignment,
  const Formulation &formulation,
  bool projectHessianPSD)
{
  if (!assignment)
    throw std::invalid_argument("DeformationModelManager: assignment must be non-null.");
  data = std::make_unique<DeformationModelManagerImpl>();
  data->assignment = std::move(assignment);
  data->mesh = data->assignment->mesh();
  data->materialFrames = data->assignment->materialFrames();
  data->elasticDefinition = data->assignment->elasticDefinition();
  data->plasticDefinition = data->assignment->plasticDefinition();
  data->elasticFixed = data->assignment->elasticFixed();
  data->plasticFixed = data->assignment->plasticFixed();
  data->parameterization = data->assignment->parameterization();
  data->parameterData = data->assignment->parameterData();
  if (!data->parameterization || !data->parameterData)
    throw std::invalid_argument(
      "DeformationModelManager requires a structural parameterization and parameter data.");
  data->nele = data->mesh->getNumElements();
  if (data->nele <= 0)
    throw std::invalid_argument("DeformationModelManager: mesh must contain at least one element.");
  validateFormulation(data->mesh->getElementType(), formulation);
  validateFrameRequirement(*data->materialFrames, data->elasticDefinition->frameRequirement(), "elastic");
  validateFrameRequirement(*data->materialFrames, data->plasticDefinition->frameRequirement(), "plastic");
  initImpl(formulation, DeformationModelConstructionOptions{ projectHessianPSD });
}

void DeformationModelManager::initImpl(
  const Formulation &formulation, DeformationModelConstructionOptions options)
{
  SPDLOG_LOGGER_INFO(
    pgo::Logging::lgr(), "Initializing element models (manager path)...");

  data->elementFEMs.resize(data->nele);
  tbb::parallel_for(0, data->nele, [&](int ele) {
    const MaterialFrame materialToReference =
      data->materialFrames->materialToReferenceFrame(ele, 0);
    std::vector<double> elasticValues(
      static_cast<std::size_t>(data->elasticFixed->numMaterialChannels()));
    std::vector<double> plasticValues(
      static_cast<std::size_t>(data->plasticFixed->numMaterialChannels()));
    data->elasticFixed->evaluate(
      ele, 0,
      std::span<const double>(
        data->parameterData->elastic.fixedValues.data(),
        static_cast<std::size_t>(data->parameterData->elastic.fixedValues.size())),
      elasticValues);
    data->plasticFixed->evaluate(
      ele, 0,
      std::span<const double>(
        data->parameterData->plastic.fixedValues.data(),
        static_cast<std::size_t>(data->parameterData->plastic.fixedValues.size())),
      plasticValues);
    auto em = data->elasticDefinition->createModelFromFixed(
      elasticValues, materialToReference);
    auto pm = data->plasticDefinition->createModelFromFixed(
      plasticValues, materialToReference);
    data->elementFEMs[ele] = formulation.createElement(
      *data->mesh, ele, std::move(em), std::move(pm), options);
  });

  if (!data->elementFEMs[0])
    throw std::runtime_error(
      "Formulation returned a null deformation model.");
  data->numElasticParams =
    data->elementFEMs[0]->getNumElasticParameters();
  data->numPlasticParams =
    data->elementFEMs[0]->getNumPlasticParameters();
  for (int ele = 1; ele < data->nele; ele++) {
    const auto &model = data->elementFEMs[ele];
    if (!model)
      throw std::runtime_error(
        "Formulation returned a null deformation model.");
    if (model->getNumElasticParameters() != data->numElasticParams ||
      model->getNumPlasticParameters() != data->numPlasticParams) {
      throw std::invalid_argument(
        "Element deformation models have inconsistent optimizable parameter channels.");
    }
  }
}

DeformationModelManager::~DeformationModelManager() = default;

int DeformationModelManager::getNumPlasticParameters() const
{
  return data->numPlasticParams;
}

int DeformationModelManager::getNumElasticParameters() const
{
  return data->numElasticParams;
}

const SimulationMesh &DeformationModelManager::getMesh() const
{
  return *data->mesh;
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

std::shared_ptr<const ElasticModelDefinition>
DeformationModelManager::elasticModelDefinition() const
{
  return data->elasticDefinition;
}

std::shared_ptr<const PlasticModelDefinition>
DeformationModelManager::plasticModelDefinition() const
{
  return data->plasticDefinition;
}

std::shared_ptr<const MaterialAssignment>
DeformationModelManager::materialAssignment() const
{
  return data->assignment;
}

MaterialFrame DeformationModelManager::materialToReferenceFrame(
  int elementId, int quadratureId) const
{
  return data->materialFrames->materialToReferenceFrame(
    elementId, quadratureId);
}

const DeformationModel &
DeformationModelManager::getDeformationModel(int eleID) const
{
  if (eleID < 0 || eleID >= data->nele)
    throw std::out_of_range(
      "DeformationModelManager: element index is out of range.");
  return *data->elementFEMs[eleID];
}

}  // namespace pgo::SolidDeformationModel
