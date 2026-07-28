#pragma once

#include "material/elastic/elasticModel1D.h"

#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

// Parameter-free one-dimensional zero energy:
//   psi(x) = 0.
class ElasticModel1DZero : public ElasticModel1D
{
public:
  ~ElasticModel1DZero() override = default;

  double compute_psi(std::span<const double> param, double x) const override;
  double compute_dpsi_dx(std::span<const double> param, double x) const override;
  double compute_d2psi_dx2(std::span<const double> param, double x) const override;

  int getNumParameters() const override { return 0; }
  double compute_dpsi_dparam(
    std::span<const double> param, int i, double x) const override;
  double compute_d2psi_dx_dparam(
    std::span<const double> param, int i, double x) const override;
  double compute_d2psi_dparam2(
    std::span<const double> param, int i, int j, double x) const override;

private:
  static void validateInputs(
    std::span<const double> param, double x);
  [[noreturn]] static void throwParameterIndexOutOfRange();
};

}  // namespace SolidDeformationModel
}  // namespace pgo
