#pragma once

#include "material/elastic/elasticModel1D.h"

#include <memory>
#include <span>

namespace pgo::SolidDeformationModel
{

// Multiplies a parameter-free one-dimensional energy by one external scale:
//   psi(param, x) = param[0] * shape(x).
//
// This is useful for a fixed material-response shape whose overall magnitude
// remains optimizable.
class ElasticModel1DScaled final : public ElasticModel1D
{
public:
  explicit ElasticModel1DScaled(
    std::shared_ptr<const ElasticModel1D> shape);

  ~ElasticModel1DScaled() override = default;

  double compute_psi(
    std::span<const double> param, double x) const override;
  double compute_dpsi_dx(
    std::span<const double> param, double x) const override;
  double compute_d2psi_dx2(
    std::span<const double> param, double x) const override;

  int getNumParameters() const override { return 1; }
  double compute_dpsi_dparam(
    std::span<const double> param, int i, double x) const override;
  double compute_d2psi_dx_dparam(
    std::span<const double> param, int i, double x) const override;
  double compute_d2psi_dparam2(
    std::span<const double> param, int i, int j, double x) const override;

private:
  static double validateAndGetScale(
    std::span<const double> param);
  static void validateParameterIndex(int i);
  static void validateX(double x);

  std::shared_ptr<const ElasticModel1D> shape_;
};

}  // namespace pgo::SolidDeformationModel
