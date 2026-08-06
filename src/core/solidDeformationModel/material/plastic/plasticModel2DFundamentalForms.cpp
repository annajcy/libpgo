#include "material/plastic/plasticModel2DFundamentalForms.h"
#include <initializer_list>
#include <stdexcept>

namespace pgo::SolidDeformationModel {
std::unique_ptr<PlasticModel> ShellPlasticity0Definition::createModel(std::span<const double> values, const MaterialFrame &) const
{
  requireFixedChannels(values, 0, id());
  return std::make_unique<PlasticModel2DFundamentalForms>();
}
}  // namespace pgo::SolidDeformationModel
