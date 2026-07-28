#pragma once

#include "material/elastic/elasticModel1D.h"

#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

// One-dimensional log-squared energy on the positive domain:
//   psi(x; lambda) = 1/2 * lambda * log(x)^2.
class ElasticModel1DLogSquared : public ElasticModel1D
{
public:
  ~ElasticModel1DLogSquared() override = default;

  double compute_psi(std::span<const double> param, double x) const override;
  double compute_dpsi_dx(std::span<const double> param, double x) const override;
  double compute_d2psi_dx2(std::span<const double> param, double x) const override;

  int getNumParameters() const override { return 1; }
  double compute_dpsi_dparam(
    std::span<const double> param, int i, double x) const override;
  double compute_d2psi_dx_dparam(
    std::span<const double> param, int i, double x) const override;
  double compute_d2psi_dparam2(
    std::span<const double> param, int i, int j, double x) const override;

private:
  static void validateParameters(std::span<const double> param);
  static void validateParameterIndex(int i);
  static void validateX(double x);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
