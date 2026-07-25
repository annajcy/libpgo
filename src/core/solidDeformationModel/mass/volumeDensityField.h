#pragma once

#include "elementScalarField.h"

#include <memory>

namespace pgo::SolidDeformationModel
{

class VolumeDensityField final
{
public:
  explicit VolumeDensityField(std::shared_ptr<const ElementScalarFieldSource> source);

  static VolumeDensityField constant(double density);
  static VolumeDensityField elementwise(EigenSupport::VXd densities);

  void validate(int numElements) const;
  double value(
    int element,
    int quadrature = 0,
    const MaterialParameterEvaluationView &state = {}) const;

private:
  std::shared_ptr<const ElementScalarFieldSource> source_;
};

}  // namespace pgo::SolidDeformationModel
