#include "koiterShellFormulation.h"

#include "deformation/shell/koiterShellElementMapping.h"

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

std::string_view KoiterShellFormulation::getName() const { return "shell_koiter"; }
int KoiterShellFormulation::numBasisFunctionsPerElement() const { return 6; }
int KoiterShellFormulation::getLocalDofs() const { return 18; }

std::unique_ptr<ShellElementMapping> KoiterShellFormulation::createElementMapping(
  const ES::V18d &restX, const std::array<bool, 6> &hasVtx) const
{
  return std::make_unique<KoiterShellElementMapping>(restX, hasVtx);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
