#pragma once

#include "material/elastic/elasticModel1D.h"

#include <memory>
#include <span>
#include <vector>

namespace pgo::SolidDeformationModel
{

// Binds all parameters of an ElasticModel1D to fixed values.
//
// The wrapped evaluator retains its x derivatives, but this adapter exposes
// zero external parameters and therefore contributes no optimizer
// coordinates.
class ElasticModel1DFixedParameters final : public ElasticModel1D
{
public:
  ElasticModel1DFixedParameters(
    std::shared_ptr<const ElasticModel1D> model,
    std::span<const double> fixedParameters);

  ElasticModel1DFixedParameters(
    std::shared_ptr<const ElasticModel1D> model,
    std::vector<double> fixedParameters);

  ~ElasticModel1DFixedParameters() override = default;

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
  static void validateExternalParameters(
    std::span<const double> param);
  [[noreturn]] static void throwParameterIndexOutOfRange();

  std::shared_ptr<const ElasticModel1D> model_;
  std::vector<double> fixedParameters_;
};

}  // namespace pgo::SolidDeformationModel
