#include "koiterShellFormulation.h"

#include "deformation/shell/koiterShellElementMapping.h"

namespace pgo
{
namespace SolidDeformationModel
{

std::string_view KoiterShellFormulation::getName() const { return "shell_koiter"; }
int KoiterShellFormulation::getNodesPerElement() const { return 6; }
int KoiterShellFormulation::getLocalDofs() const { return 18; }

std::unique_ptr<ShellElementMapping> KoiterShellFormulation::createElementMapping(
  const double restX[18], const bool hasVtx[6]) const
{
  return std::make_unique<KoiterShellElementMapping>(restX, hasVtx);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
