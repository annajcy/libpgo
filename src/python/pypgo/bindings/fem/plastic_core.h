#pragma once

#include "deformationModelManager.h"

#include <memory>
#include <string>

namespace pgo
{

// Python-facing plastic model wrapper.  Holds a DeformationModelPlasticMaterial
// enum plus the DOF count.
class PyPlasticModel
{
public:
  PyPlasticModel(SolidDeformationModel::DeformationModelPlasticMaterial type, int dofs);

  std::string name() const;
  SolidDeformationModel::DeformationModelPlasticMaterial type() const { return type_; }
  int dofs() const { return dofs_; }

private:
  SolidDeformationModel::DeformationModelPlasticMaterial type_;
  int dofs_;
};

// --- factory functions ---

std::shared_ptr<PyPlasticModel> make_volumetric_plasticity(int dofs = 6);
std::shared_ptr<PyPlasticModel> make_shell_plasticity(int dofs = 1);

}  // namespace pgo
