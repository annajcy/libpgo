#include "material/elastic/elasticModel1DLogSquared.h"

#include <cmath>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{

void ElasticModel1DLogSquared::validateParameters(
  std::span<const double> param)
{
  if (param.size() != 1)
    throw std::invalid_argument(
      "ElasticModel1DLogSquared expects one material parameter");
  if (!std::isfinite(param[0]))
    throw std::invalid_argument(
      "ElasticModel1DLogSquared parameter must be finite");
}

void ElasticModel1DLogSquared::validateParameterIndex(int i)
{
  if (i != 0)
    throw std::out_of_range(
      "ElasticModel1DLogSquared parameter index is out of range");
}

void ElasticModel1DLogSquared::validateX(double x)
{
  if (!std::isfinite(x) || !(x > 0.0))
    throw std::domain_error(
      "ElasticModel1DLogSquared deformation variable must be "
      "finite and positive");
}

double ElasticModel1DLogSquared::compute_psi(
  std::span<const double> param, double x) const
{
  validateParameters(param);
  validateX(x);
  const double logX = std::log(x);
  return 0.5 * param[0] * logX * logX;
}

double ElasticModel1DLogSquared::compute_dpsi_dx(
  std::span<const double> param, double x) const
{
  validateParameters(param);
  validateX(x);
  return param[0] * std::log(x) / x;
}

double ElasticModel1DLogSquared::compute_d2psi_dx2(
  std::span<const double> param, double x) const
{
  validateParameters(param);
  validateX(x);
  return param[0] * (1.0 - std::log(x)) / (x * x);
}

double ElasticModel1DLogSquared::compute_dpsi_dparam(
  std::span<const double> param, int i, double x) const
{
  validateParameters(param);
  validateParameterIndex(i);
  validateX(x);
  const double logX = std::log(x);
  return 0.5 * logX * logX;
}

double ElasticModel1DLogSquared::compute_d2psi_dx_dparam(
  std::span<const double> param, int i, double x) const
{
  validateParameters(param);
  validateParameterIndex(i);
  validateX(x);
  return std::log(x) / x;
}

double ElasticModel1DLogSquared::compute_d2psi_dparam2(
  std::span<const double> param,
  int i,
  int j,
  double x) const
{
  validateParameters(param);
  validateParameterIndex(i);
  validateParameterIndex(j);
  validateX(x);
  return 0.0;
}

}  // namespace pgo::SolidDeformationModel
