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
  std::shared_ptr<const MaterialBinding> binding;
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

}  // namespace

DeformationModelManager::DeformationModelManager(
  std::shared_ptr<const SimulationMesh> mesh,
  std::shared_ptr<const MaterialBinding> materialBinding,
  const Formulation &formulation,
  bool projectHessianPSD)
{
  if (!mesh || !materialBinding)
    throw std::invalid_argument(
      "DeformationModelManager requires a mesh and material binding.");
  if (materialBinding->numElements() != mesh->getNumElements())
    throw std::invalid_argument(
      "DeformationModelManager material binding element count does not match mesh.");
  data = std::make_unique<DeformationModelManagerImpl>();
  data->mesh = std::move(mesh);
  data->binding = std::move(materialBinding);
  data->nele = data->mesh->getNumElements();
  if (data->nele <= 0)
    throw std::invalid_argument(
      "DeformationModelManager: mesh must contain at least one element.");
  validateFormulation(data->mesh->getElementType(), formulation);
  initImpl(formulation, DeformationModelConstructionOptions{ projectHessianPSD });
}

void DeformationModelManager::initImpl(
  const Formulation &formulation, DeformationModelConstructionOptions options)
{
  SPDLOG_LOGGER_INFO(
    pgo::Logging::lgr(), "Initializing element models (manager path)...");

  const auto &elastic = data->binding->elastic();
  const auto &plastic = data->binding->plastic();
  const EigenSupport::VXd &elasticFixedValues = elastic.fixed().values();
  const EigenSupport::VXd &plasticFixedValues = plastic.fixed().values();

  data->elementFEMs.resize(data->nele);
  tbb::parallel_for(0, data->nele, [&](int ele) {
    const MaterialFrame materialToReference =
      data->binding->materialFrames()->materialToReferenceFrame(ele, 0);
    std::vector<double> elasticValues(
      static_cast<std::size_t>(elastic.fixed().field()->numMaterialChannels()));
    std::vector<double> plasticValues(
      static_cast<std::size_t>(plastic.fixed().field()->numMaterialChannels()));
    elastic.fixed().field()->evaluate(
      ele, 0,
      std::span<const double>(
        elasticFixedValues.data(),
        static_cast<std::size_t>(elasticFixedValues.size())),
      elasticValues);
    plastic.fixed().field()->evaluate(
      ele, 0,
      std::span<const double>(
        plasticFixedValues.data(),
        static_cast<std::size_t>(plasticFixedValues.size())),
      plasticValues);
    auto em = elastic.definition()->createModel(
      elasticValues, materialToReference);
    auto pm = plastic.definition()->createModel(
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
  return *data->binding->materialFrames();
}

std::shared_ptr<const MaterialFrameField>
DeformationModelManager::materialFrameFieldPtr() const
{
  return data->binding->materialFrames();
}

std::shared_ptr<const ElasticModelDefinition>
DeformationModelManager::elasticModelDefinition() const
{
  return data->binding->elastic().definition();
}

std::shared_ptr<const PlasticModelDefinition>
DeformationModelManager::plasticModelDefinition() const
{
  return data->binding->plastic().definition();
}

std::shared_ptr<const OptimizableParameterField>
DeformationModelManager::elasticOptimizableField() const
{
  return data->binding->elastic().optimizableField();
}

std::shared_ptr<const OptimizableParameterField>
DeformationModelManager::plasticOptimizableField() const
{
  return data->binding->plastic().optimizableField();
}

std::shared_ptr<const MaterialBinding>
DeformationModelManager::materialBinding() const
{
  return data->binding;
}

MaterialFrame DeformationModelManager::materialToReferenceFrame(
  int elementId, int quadratureId) const
{
  return data->binding->materialFrames()->materialToReferenceFrame(
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
