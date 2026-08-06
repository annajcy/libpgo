#include "core.h"

namespace pgo
{
namespace SD = SolidDeformationModel;

PySystematicPokingDefinition::PySystematicPokingDefinition(
  const std::vector<double> &stretchKnots,
  int stretchRestKnotIndex,
  const std::vector<double> &volumeKnots,
  int volumeRestKnotIndex):
  PyElasticModelDefinition(
    std::make_shared<SD::SystematicPokingDefinition>(
      stretchKnots,
      stretchRestKnotIndex,
      volumeKnots,
      volumeRestKnotIndex))
{
}

}  // namespace pgo
