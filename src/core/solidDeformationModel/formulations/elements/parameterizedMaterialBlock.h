#pragma once

namespace pgo
{
namespace SolidDeformationModel
{

class ElasticModel;
class PlasticModel;
class ParameterField;

struct ElasticBlock
{
  ElasticModel *model = nullptr;
  const ParameterField *parameters = nullptr;
};

struct PlasticBlock
{
  PlasticModel *model = nullptr;
  const ParameterField *parameters = nullptr;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
