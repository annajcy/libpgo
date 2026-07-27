#pragma once

namespace pgo::SolidDeformationModel
{

class MaterialParameterization;
struct MaterialParameterData;

void validateMaterialParameterData(
  const MaterialParameterization &parameterization,
  const MaterialParameterData &data);

}  // namespace pgo::SolidDeformationModel
