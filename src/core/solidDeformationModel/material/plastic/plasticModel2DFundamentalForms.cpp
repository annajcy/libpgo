#include "material/plastic/plasticModel2DFundamentalForms.h"
#include <initializer_list>
#include <stdexcept>

namespace pgo::SolidDeformationModel {
int ShellPlasticity0Definition::numOptimizableChannels() const { return 0; }
std::unique_ptr<PlasticModel> ShellPlasticity0Definition::createModel(std::span<const double> values, const MaterialFrame &) const
{
  if (!values.empty()) throw std::invalid_argument("shell_ff_dof0 has no fixed channels");
  return std::make_unique<PlasticModel2DFundamentalForms>();
}
}  // namespace pgo::SolidDeformationModel
