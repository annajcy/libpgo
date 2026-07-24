#pragma once

namespace pgo
{
namespace SolidDeformationModel
{
class PlasticModel
{
public:
  PlasticModel() {}
  virtual ~PlasticModel() {}

  virtual int getNumParameters() const { return 0; }

  // Write default (rest-configuration) plastic parameters into param.
  // No-op for zero-param models.
  virtual void defaultParams(double * /*param*/) const {}
};
}  // namespace SolidDeformationModel
}  // namespace pgo
