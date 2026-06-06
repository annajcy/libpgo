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

  // Called when the element's fiber axes change. No-op for models that
  // don't track a fiber direction.
  virtual void setFiberAxes(const double * /*R*/) {}

  // Returns true when this model does not depend on fiber direction, so the
  // element's aligned matrix should be reported as identity.
  virtual bool isIdentityTransform() const { return false; }

  // Write default (rest-configuration) plastic parameters into param.
  // No-op for zero-param models.
  virtual void defaultParams(double * /*param*/) const {}
};
}  // namespace SolidDeformationModel
}  // namespace pgo