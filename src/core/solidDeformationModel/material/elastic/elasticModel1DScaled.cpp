#include "material/elastic/elasticModel1DScaled.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace pgo::SolidDeformationModel
{

ElasticModel1DScaled::ElasticModel1DScaled(
  std::shared_ptr<const ElasticModel1D> shape):
  shape_(std::move(shape))
{
  if (!shape_)
    throw std::invalid_argument(
      "ElasticModel1DScaled requires a non-null shape");
  if (shape_->getNumParameters() != 0)
    throw std::invalid_argument(
      "ElasticModel1DScaled shape must expose zero parameters");
}

double ElasticModel1DScaled::validateAndGetScale(
  std::span<const double> param)
{
  if (param.size() != 1)
    throw std::invalid_argument(
      "ElasticModel1DScaled expects exactly one scale parameter");
  if (!std::isfinite(param[0]))
    throw std::invalid_argument(
      "ElasticModel1DScaled scale parameter must be finite");
  return param[0];
}

void ElasticModel1DScaled::validateParameterIndex(int i)
{
  if (i != 0)
    throw std::out_of_range(
      "ElasticModel1DScaled parameter index must be zero");
}

void ElasticModel1DScaled::validateX(double x)
{
  if (!std::isfinite(x))
    throw std::invalid_argument(
      "ElasticModel1DScaled deformation variable must be finite");
}

double ElasticModel1DScaled::compute_psi(
  std::span<const double> param, double x) const
{
  return validateAndGetScale(param) *
    shape_->compute_psi({}, x);
}

double ElasticModel1DScaled::compute_dpsi_dx(
  std::span<const double> param, double x) const
{
  return validateAndGetScale(param) *
    shape_->compute_dpsi_dx({}, x);
}

double ElasticModel1DScaled::compute_d2psi_dx2(
  std::span<const double> param, double x) const
{
  return validateAndGetScale(param) *
    shape_->compute_d2psi_dx2({}, x);
}

double ElasticModel1DScaled::compute_dpsi_dparam(
  std::span<const double> param, int i, double x) const
{
  validateAndGetScale(param);
  validateParameterIndex(i);
  return shape_->compute_psi({}, x);
}

double ElasticModel1DScaled::compute_d2psi_dx_dparam(
  std::span<const double> param, int i, double x) const
{
  validateAndGetScale(param);
  validateParameterIndex(i);
  return shape_->compute_dpsi_dx({}, x);
}

double ElasticModel1DScaled::compute_d2psi_dparam2(
  std::span<const double> param,
  int i,
  int j,
  double x) const
{
  validateAndGetScale(param);
  validateParameterIndex(i);
  validateParameterIndex(j);
  validateX(x);
  return 0.0;
}

}  // namespace pgo::SolidDeformationModel
