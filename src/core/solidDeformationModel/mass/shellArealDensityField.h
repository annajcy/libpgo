#pragma once

#include "elementScalarField.h"

#include <memory>

namespace pgo::SolidDeformationModel
{

class ShellArealDensityField final
{
public:
  explicit ShellArealDensityField(
    std::shared_ptr<const ElementScalarFieldSource> source);

  static ShellArealDensityField constant(double arealDensity);
  static ShellArealDensityField elementwise(EigenSupport::VXd arealDensities);
  static ShellArealDensityField fromDensityThickness(
    double density, double thickness);
  static ShellArealDensityField fromDensityThickness(
    double density, EigenSupport::VXd thickness);

  void validate(int numElements) const;
  double value(int element, int quadrature = 0) const;

private:
  std::shared_ptr<const ElementScalarFieldSource> source_;
};

}  // namespace pgo::SolidDeformationModel
