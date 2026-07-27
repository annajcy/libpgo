#pragma once

#include "material/model/plasticModelDefinition.h"
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
  std::vector<std::string> fixedChannelNames() const;
  std::vector<std::string> optimizableChannelNames() const;
  std::shared_ptr<const SolidDeformationModel::PlasticModelDefinition> definition() const { return definition_; }

protected:
  std::shared_ptr<const SolidDeformationModel::PlasticModelDefinition> definition_;
  int dofs_;
};

class PyVolumetricPlasticity0Definition final : public PyPlasticModelDefinition { public: PyVolumetricPlasticity0Definition(); };
class PyVolumetricPlasticity3Definition final : public PyPlasticModelDefinition { public: PyVolumetricPlasticity3Definition(); };
class PyVolumetricPlasticity6Definition final : public PyPlasticModelDefinition { public: PyVolumetricPlasticity6Definition(); };
class PyShellPlasticity0Definition final : public PyPlasticModelDefinition { public: PyShellPlasticity0Definition(); };
class PyShellPlasticity1Definition final : public PyPlasticModelDefinition { public: PyShellPlasticity1Definition(); };

}  // namespace pgo
