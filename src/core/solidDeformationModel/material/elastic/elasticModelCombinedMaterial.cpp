#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModelInvariantBasedMaterial.h"
#include "material/elastic/invariantBasedMaterialStVK.h"
#include "material/elastic/elasticModelVolumeMaterial.h"
#include "simulation/simulationMesh.h"
#include <stdexcept>
#include <string>

namespace pgo::SolidDeformationModel {
namespace {
const SimulationMeshENuMaterial &enuMaterial(const SimulationMesh &mesh, int element) {
  return mesh.requireElementField<SimulationMeshENuMaterial>().at(element);
}
void expectSize(std::span<double> output, std::size_t expected) {
  if (output.size() != expected) throw std::invalid_argument("elastic config default parameter buffer has the wrong size");
}
// This model has no optimization channels.
}

std::span<const std::string_view> StVKVolumeConfig::parameterChannelNames() const { return {}; }
void StVKVolumeConfig::initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double> output) const { expectSize(output, 0); }
std::unique_ptr<ElasticModel> StVKVolumeConfig::createModel(const SimulationMesh &mesh, int element, const MaterialFrame &) const
{
  const auto &mat = enuMaterial(mesh, element);
  return std::make_unique<ElasticModelCombinedMaterial<2>>(
    std::make_unique<ElasticModelInvariantBasedMaterial>(
      std::make_unique<InvariantBasedMaterialStVK>(mat.getE(), mat.getNu(), mat.getCompressionRatio())),
    std::make_unique<ElasticModelVolumeMaterial>(mat.getCompressionRatio()));
}
}  // namespace pgo::SolidDeformationModel
