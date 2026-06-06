#include "formulation_core.h"

namespace pgo
{

std::shared_ptr<PyVolumetricFormulation> make_tet_p1()
{
  return std::make_shared<PyVolumetricFormulation>(
    std::make_shared<SolidDeformationModel::P1TetFormulation>());
}

std::shared_ptr<PyVolumetricFormulation> make_linear_cubic()
{
  return std::make_shared<PyVolumetricFormulation>(
    std::make_shared<SolidDeformationModel::LinearCubicFormulation>());
}

std::shared_ptr<PyVolumetricFormulation> make_tricubic_hermite()
{
  return std::make_shared<PyVolumetricFormulation>(
    std::make_shared<SolidDeformationModel::TricubicHermiteFormulation>());
}

std::shared_ptr<PyShellFormulation> make_koiter_shell()
{
  return std::make_shared<PyShellFormulation>(
    std::make_shared<SolidDeformationModel::KoiterShellFormulation>());
}

}  // namespace pgo
