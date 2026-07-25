#include "material/plastic/plasticModel2DFundamentalForms.h"
#include "simulation/simulationMesh.h"
#include <initializer_list>
#include <stdexcept>

namespace pgo::SolidDeformationModel {
namespace {
void expectSize(std::span<double> output, std::size_t expected) {
  if (output.size() != expected) throw std::invalid_argument("plastic config default parameter buffer has the wrong size");
}
}

std::span<const std::string_view> ShellPlasticity0Config::parameterChannelNames() const { return {}; }
void ShellPlasticity0Config::initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double> output) const { expectSize(output, 0); }
std::unique_ptr<PlasticModel> ShellPlasticity0Config::createModel(const SimulationMesh &, int, const MaterialFrame &) const
{
  return std::make_unique<PlasticModel2DFundamentalForms>();
}
}  // namespace pgo::SolidDeformationModel
