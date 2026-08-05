#pragma once

#include "EigenDef.h"

#include <memory>

namespace pgo::SolidDeformationModel
{

// A scalar field sampled on element/quadrature locations.  The interface is
// deliberately independent of volume/shell units; those semantics are added
// by VolumeDensityField and ShellArealDensityField.
class ElementScalarFieldSource
{
public:
  virtual ~ElementScalarFieldSource() = default;

  virtual void validate(int numElements) const = 0;

  virtual double value(int element, int quadrature) const = 0;
};

class ConstantScalarFieldSource final : public ElementScalarFieldSource
{
public:
  explicit ConstantScalarFieldSource(double value);

  void validate(int numElements) const override;
  double value(int element, int quadrature) const override;

private:
  double value_ = 0.0;
};

class ElementwiseScalarFieldSource final : public ElementScalarFieldSource
{
public:
  explicit ElementwiseScalarFieldSource(EigenSupport::VXd values);

  void validate(int numElements) const override;
  double value(int element, int quadrature) const override;

private:
  EigenSupport::VXd values_;
};

}  // namespace pgo::SolidDeformationModel
