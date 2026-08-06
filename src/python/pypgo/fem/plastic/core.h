#pragma once

#include "material/plastic/plasticModelDefinition.h"
#include "material/plastic/plasticModel3DConstant.h"
#include "material/plastic/plasticModel3D3DOF.h"
#include "material/plastic/plasticModel3D6DOF.h"
#include "material/plastic/plasticModel2DFundamentalForms.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"

#include <memory>
#include <string>
#include <vector>

namespace pgo
{

// Python-facing immutable plastic model definition wrapper.
class PyPlasticModelDefinition
{
public:
  PyPlasticModelDefinition(
    std::shared_ptr<const SolidDeformationModel::PlasticModelDefinition> definition,
    int dofs): definition_(std::move(definition)), dofs_(dofs) {}

  std::string name() const { return std::string(definition_->id()); }
  int dofs() const { return dofs_; }
  int numFixedChannels() const { return definition_->numFixedChannels(); }
  int numOptimizableChannels() const
  {
    return definition_->numOptimizableChannels();
  }
  std::shared_ptr<const SolidDeformationModel::PlasticModelDefinition> definition() const { return definition_; }

protected:
  std::shared_ptr<const SolidDeformationModel::PlasticModelDefinition> definition_;
  int dofs_;
};

// Default-constructible wrapper around one concrete C++ definition.  Each
// template instantiation is a distinct C++ type, so nanobind can register it
// as its own Python class while sharing the base wrapper implementation.
template<int Dofs, class DefinitionT>
class PyPlasticModelDefinitionT final : public PyPlasticModelDefinition
{
public:
  PyPlasticModelDefinitionT():
    PyPlasticModelDefinition(std::make_shared<DefinitionT>(), Dofs)
  {
  }
};

using PyVolumetricPlasticity0Definition =
  PyPlasticModelDefinitionT<0, SolidDeformationModel::VolumetricPlasticity0Definition>;
using PyVolumetricPlasticity3Definition =
  PyPlasticModelDefinitionT<3, SolidDeformationModel::VolumetricPlasticity3Definition>;
using PyVolumetricPlasticity6Definition =
  PyPlasticModelDefinitionT<6, SolidDeformationModel::VolumetricPlasticity6Definition>;
using PyShellPlasticity0Definition =
  PyPlasticModelDefinitionT<0, SolidDeformationModel::ShellPlasticity0Definition>;
using PyShellPlasticity1Definition =
  PyPlasticModelDefinitionT<1, SolidDeformationModel::ShellPlasticity1Definition>;

}  // namespace pgo
