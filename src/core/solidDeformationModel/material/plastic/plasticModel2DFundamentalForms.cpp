#include "material/plastic/plasticModel2DFundamentalForms.h"
#include <initializer_list>
#include <stdexcept>

namespace pgo::SolidDeformationModel {
MaterialChannelSchema ShellPlasticity0Definition::optimizableChannelSchema() const { return {}; }
std::unique_ptr<PlasticModel> ShellPlasticity0Definition::createModelFromFixed(std::span<const double> values, const MaterialFrame &) const
{
  if (!values.empty()) throw std::invalid_argument("shell_ff_dof0 has no fixed channels");
  return std::make_unique<PlasticModel2DFundamentalForms>();
}
}  // namespace pgo::SolidDeformationModel
