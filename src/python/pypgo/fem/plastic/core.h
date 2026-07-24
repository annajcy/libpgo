#pragma once

#include "material/plastic/plasticModel.h"
#include "material/plastic/plasticModel3DConstant.h"
#include "material/plastic/plasticModel3D3DOF.h"
#include "material/plastic/plasticModel3D6DOF.h"
#include "material/plastic/plasticModel2DFundamentalForms.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"

#include <memory>
#include <string>

namespace pgo
{

// Python-facing immutable plastic model configuration wrapper.
class PyPlasticModelConfig
{
public:
  PyPlasticModelConfig(
    std::shared_ptr<const SolidDeformationModel::PlasticModelConfig> config,
    int dofs): config_(std::move(config)), dofs_(dofs) {}

  std::string name() const { return std::string(config_->id()); }
  int dofs() const { return dofs_; }
  std::shared_ptr<const SolidDeformationModel::PlasticModelConfig> config() const { return config_; }

protected:
  std::shared_ptr<const SolidDeformationModel::PlasticModelConfig> config_;
  int dofs_;
};

class PyVolumetricPlasticity0Config final : public PyPlasticModelConfig { public: PyVolumetricPlasticity0Config(); };
class PyVolumetricPlasticity3Config final : public PyPlasticModelConfig { public: PyVolumetricPlasticity3Config(); };
class PyVolumetricPlasticity6Config final : public PyPlasticModelConfig { public: PyVolumetricPlasticity6Config(); };
class PyShellPlasticity0Config final : public PyPlasticModelConfig { public: PyShellPlasticity0Config(); };
class PyShellPlasticity1Config final : public PyPlasticModelConfig { public: PyShellPlasticity1Config(); };

}  // namespace pgo
