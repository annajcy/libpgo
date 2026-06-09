#include "core.h"

#include "material/plastic/plasticModelFactory.h"

namespace pgo
{
namespace SD = SolidDeformationModel;

PyPlasticModel::PyPlasticModel(SD::DeformationModelPlasticMaterial type, int dofs)
  : type_(type), dofs_(dofs)
{
}

std::string PyPlasticModel::name() const
{
  return SD::PlasticModelFactory::modelId(type_);
}

std::shared_ptr<PyPlasticModel> make_volumetric_plasticity(int dofs)
{
  SD::DeformationModelPlasticMaterial type;
  switch (dofs) {
    case 0: type = SD::DeformationModelPlasticMaterial::VOLUMETRIC_DOF0; break;
    case 3: type = SD::DeformationModelPlasticMaterial::VOLUMETRIC_DOF3; break;
    default: type = SD::DeformationModelPlasticMaterial::VOLUMETRIC_DOF6; break;
  }
  return std::make_shared<PyPlasticModel>(type, dofs);
}

std::shared_ptr<PyPlasticModel> make_shell_plasticity(int dofs)
{
  SD::DeformationModelPlasticMaterial type =
    (dofs == 0) ? SD::DeformationModelPlasticMaterial::SHELL_FF_DOF0
                : SD::DeformationModelPlasticMaterial::SHELL_FF_DOF1;
  return std::make_shared<PyPlasticModel>(type, dofs);
}

}  // namespace pgo
