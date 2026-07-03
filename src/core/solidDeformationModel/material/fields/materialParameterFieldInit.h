/*
author: Bohan Wang
copyright to USC, MIT, NUS
*/

#pragma once

#include "material/materialTypes.h"
#include "EigenSupport.h"

#include <memory>
#include <optional>

namespace pgo
{
namespace SolidDeformationModel
{

class OptimizableField;
class SimulationMesh;

enum class ElasticMaterialFieldType
{
  ELEMENTWISE,
  CONSTANT,
};

enum class PlasticMaterialFieldType
{
  ELEMENTWISE,
  CONSTANT,
};

struct ElasticFieldInit
{
  ElasticMaterialFieldType type = ElasticMaterialFieldType::ELEMENTWISE;
  std::optional<EigenSupport::VXd> values;
};

struct PlasticFieldInit
{
  PlasticMaterialFieldType type = PlasticMaterialFieldType::ELEMENTWISE;
  std::optional<EigenSupport::VXd> values;
};

std::shared_ptr<OptimizableField> createElasticParameterField(
  const SimulationMesh &mesh,
  DeformationModelElasticMaterial elastic,
  ElasticFieldInit fieldInit);

std::shared_ptr<OptimizableField> createPlasticParameterField(
  const SimulationMesh &mesh,
  DeformationModelPlasticMaterial plastic,
  PlasticFieldInit fieldInit);

}  // namespace SolidDeformationModel
}  // namespace pgo
