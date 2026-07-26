#pragma once

#include "material/elastic/elasticModel1D.h"

#include <cmath>
#include <span>
#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

// One-dimensional quadratic energy:
//   psi(x; p) = 1/2 * coeff * p[0] * x^2.
class ElasticModel1DQuadratic : public ElasticModel1D
{
public:
  explicit ElasticModel1DQuadratic(double coeff):
    coeff_(coeff)
  {
    if (!std::isfinite(coeff_))
      throw std::invalid_argument("ElasticModel1DQuadratic coefficient must be finite");
  }

  ~ElasticModel1DQuadratic() override = default;

  double compute_psi(std::span<const double> param, double x) const override;
  double compute_dpsi_dx(std::span<const double> param, double x) const override;
  double compute_d2psi_dx2(std::span<const double> param, double x) const override;

  int getNumParameters() const override { return 1; }
  double compute_dpsi_dparam(std::span<const double> param, int i, double x) const override;
  double compute_d2psi_dx_dparam(std::span<const double> param, int i, double x) const override;
  double compute_d2psi_dparam2(
    std::span<const double> param, int i, int j, double x) const override;

private:
  static void validateParameters(std::span<const double> param);
  static void validateParameterIndex(int i);
  static void validateX(double x);

  double coeff_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
