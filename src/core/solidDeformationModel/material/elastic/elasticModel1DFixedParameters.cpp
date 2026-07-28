#include "material/elastic/elasticModel1DFixedParameters.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace pgo::SolidDeformationModel
{

ElasticModel1DFixedParameters::ElasticModel1DFixedParameters(
  std::shared_ptr<const ElasticModel1D> model,
  std::span<const double> fixedParameters):
  ElasticModel1DFixedParameters(
    std::move(model),
    std::vector<double>(
      fixedParameters.begin(), fixedParameters.end()))
{
}

ElasticModel1DFixedParameters::ElasticModel1DFixedParameters(
  std::shared_ptr<const ElasticModel1D> model,
  std::vector<double> fixedParameters):
  model_(std::move(model)),
  fixedParameters_(std::move(fixedParameters))
{
  if (!model_)
    throw std::invalid_argument(
      "ElasticModel1DFixedParameters requires a non-null model");
  const int expectedParameterCount = model_->getNumParameters();
  if (expectedParameterCount < 0)
    throw std::invalid_argument(
      "ElasticModel1DFixedParameters wrapped model returned a "
      "negative parameter count");
  if (fixedParameters_.size() !=
    static_cast<std::size_t>(expectedParameterCount))
    throw std::invalid_argument(
      "ElasticModel1DFixedParameters fixed parameter count does "
      "not match the wrapped model");
  for (double value : fixedParameters_) {
    if (!std::isfinite(value))
      throw std::invalid_argument(
        "ElasticModel1DFixedParameters fixed parameters must be finite");
  }
}

void ElasticModel1DFixedParameters::validateExternalParameters(
  std::span<const double> param)
{
  if (!param.empty())
    throw std::invalid_argument(
      "ElasticModel1DFixedParameters expects no external parameters");
}

void ElasticModel1DFixedParameters::throwParameterIndexOutOfRange()
{
  throw std::out_of_range(
    "ElasticModel1DFixedParameters has no external parameters");
}

double ElasticModel1DFixedParameters::compute_psi(
  std::span<const double> param, double x) const
{
  validateExternalParameters(param);
  return model_->compute_psi(fixedParameters_, x);
}

double ElasticModel1DFixedParameters::compute_dpsi_dx(
  std::span<const double> param, double x) const
{
  validateExternalParameters(param);
  return model_->compute_dpsi_dx(fixedParameters_, x);
}

double ElasticModel1DFixedParameters::compute_d2psi_dx2(
  std::span<const double> param, double x) const
{
  validateExternalParameters(param);
  return model_->compute_d2psi_dx2(fixedParameters_, x);
}

double ElasticModel1DFixedParameters::compute_dpsi_dparam(
  std::span<const double> param, int i, double x) const
{
  validateExternalParameters(param);
  (void)i;
  (void)x;
  throwParameterIndexOutOfRange();
}

double ElasticModel1DFixedParameters::compute_d2psi_dx_dparam(
  std::span<const double> param, int i, double x) const
{
  validateExternalParameters(param);
  (void)i;
  (void)x;
  throwParameterIndexOutOfRange();
}

double ElasticModel1DFixedParameters::compute_d2psi_dparam2(
  std::span<const double> param,
  int i,
  int j,
  double x) const
{
  validateExternalParameters(param);
  (void)i;
  (void)j;
  (void)x;
  throwParameterIndexOutOfRange();
}

}  // namespace pgo::SolidDeformationModel
