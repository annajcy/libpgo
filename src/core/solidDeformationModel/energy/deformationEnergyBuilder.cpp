/*
author: Bohan Wang
copyright to USC
*/
#include "energy/deformationEnergyBuilder.h"

#include "simulation/simulationMesh.h"
#include "deformation/deformationModelAssembler.h"
#include "material/core/materialParameterBuilder.h"
#include "pgoLogging.h"

#include <utility>
#include <algorithm>
#include <string>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace ES = pgo::EigenSupport;

namespace
{
void validateBuilderInputs(
  const std::shared_ptr<const SimulationMesh> &mesh,
  const std::shared_ptr<const ElasticModelConfig> &elastic,
  const std::shared_ptr<const PlasticModelConfig> &plastic)
{
  if (!mesh)
    throw std::invalid_argument("makeDeformationEnergy: mesh must be non-null.");
  if (!elastic)
    throw std::invalid_argument("makeDeformationEnergy: elastic config must be non-null.");
  if (!plastic)
    throw std::invalid_argument("makeDeformationEnergy: plastic config must be non-null.");
}
}  // namespace

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  std::shared_ptr<const SimulationMesh> mesh,
  std::shared_ptr<const ElasticModelConfig> elastic,
  std::shared_ptr<const PlasticModelConfig> plastic,
  std::shared_ptr<MaterialParameters> materialParameters,
  std::shared_ptr<const MaterialFrameField> materialFrames,
  const Formulation &formulation,
  const DeformationModelOptions &opts)
{
  validateBuilderInputs(mesh, elastic, plastic);
  const int nele = mesh->getNumElements();

  SPDLOG_LOGGER_INFO(pgo::Logging::lgr(), "Building deformation energy with formulation: {}",
    formulation.getName());

  if (!materialParameters)
    throw std::invalid_argument(
      "makeDeformationEnergy: materialParameters must be non-null.");
  if (!materialFrames)
    throw std::invalid_argument(
      "makeDeformationEnergy: materialFrames must be non-null.");
  const auto checkChannels = [](const auto &field, const auto &spec, const char *name) {
    const auto names = field.channelNames();
    if (names.size() != spec.size() ||
      !std::equal(names.begin(), names.end(), spec.begin()))
      throw std::invalid_argument(std::string("makeDeformationEnergy: ") + name +
        " parameter space schema does not match the config.");
  };
  checkChannels(materialParameters->space()->elastic(), elastic->parameterChannelNames(), "elastic");
  checkChannels(materialParameters->space()->plastic(), plastic->parameterChannelNames(), "plastic");
  if (materialParameters->space()->elastic().dofLayout().numElements() != nele ||
    materialParameters->space()->plastic().dofLayout().numElements() != nele)
    throw std::invalid_argument("makeDeformationEnergy: parameter space element count does not match mesh.");

  auto manager = std::make_shared<DeformationModelManager>(
    mesh, elastic, plastic, formulation,
    opts.projectHessianPSD,
    materialFrames);

  ES::VXd elementWeights = opts.elementWeights;
  if (elementWeights.size() == 0)
    elementWeights = ES::VXd::Ones(nele);
  else if (static_cast<int>(elementWeights.size()) != nele)
    throw std::invalid_argument("makeDeformationEnergy: elementWeights size does not match the element count.");

  auto assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation,
    materialParameters->space(),
    std::span<const double>(elementWeights.data(), static_cast<std::size_t>(elementWeights.size())));

  auto energy = std::make_shared<DeformationModelEnergy>(
    std::move(assembler), std::move(materialParameters),
    0, opts.enableMaterialMaxStep);

  return energy;
}

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(
  std::shared_ptr<const SimulationMesh> mesh,
  std::shared_ptr<const ElasticModelConfig> elastic,
  std::shared_ptr<const PlasticModelConfig> plastic,
  const Formulation &formulation,
  const DeformationModelOptions &opts)
{
  validateBuilderInputs(mesh, elastic, plastic);
  auto materialParameters =
    makeDefaultMaterialParameters(*mesh, *elastic, *plastic);
  auto materialFrames =
    makeGlobalAxesMaterialFrameField(mesh->getNumElements());
  return makeDeformationEnergy(
    std::move(mesh), elastic, plastic,
    std::move(materialParameters), std::move(materialFrames),
    formulation, opts);
}

}  // namespace pgo::SolidDeformationModel
