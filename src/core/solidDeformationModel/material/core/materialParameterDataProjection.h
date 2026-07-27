#pragma once

#include "materialParameterData.h"
#include "materialParameterDataValidation.h"
#include "materialParameterization.h"
#include "simulation/importedMaterial.h"

#include <memory>
#include <span>
#include <string_view>
#include <vector>

namespace pgo::SolidDeformationModel
{

class MaterialParameterDataProjection
{
public:
  virtual ~MaterialParameterDataProjection() = default;

  MaterialParameterData project(
    const ImportedMaterialData &source,
    const MaterialParameterization &target) const
  {
    MaterialParameterData data = convert(source, target);
    validateMaterialParameterData(target, data);
    return data;
  }

public:
  /// Resolve one named raw input for every source element.
  static std::vector<double> resolveNamedInput(
    const ImportedMaterialData &source,
    std::string_view name);

  /// Pack element-major local inputs into a field's global parameter vector.
  /// Shared global DOFs are required to receive consistent values.
  static EigenSupport::VXd packElementInputs(
    const ParameterLayout &layout,
    std::span<const double> elementLocalValues);

protected:
  virtual MaterialParameterData convert(
    const ImportedMaterialData &source,
    const MaterialParameterization &target) const = 0;
};

/// Resolve parameter inputs by exact channel name.  Values may come from an
/// imported spatial field or from scalar properties on material records.
class NamedChannelMaterialParameterDataProjection final :
  public MaterialParameterDataProjection
{
protected:
  MaterialParameterData convert(
    const ImportedMaterialData &source,
    const MaterialParameterization &target) const override;
};

}  // namespace pgo::SolidDeformationModel
