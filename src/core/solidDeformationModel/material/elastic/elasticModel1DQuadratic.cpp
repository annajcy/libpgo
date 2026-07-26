#include "material/elastic/elasticModel1DQuadratic.h"

namespace pgo::SolidDeformationModel
{

void ElasticModel1DQuadratic::validateParameters(std::span<const double> param)
{
  if (param.size() != 1)
    throw std::invalid_argument("ElasticModel1DQuadratic expects one material parameter");
  if (!std::isfinite(param[0]))
    throw std::invalid_argument("ElasticModel1DQuadratic parameter must be finite");
}

void ElasticModel1DQuadratic::validateParameterIndex(int i)
{
  if (i != 0)
    throw std::out_of_range("ElasticModel1DQuadratic parameter index is out of range");
}

void ElasticModel1DQuadratic::validateX(double x)
{
  if (!std::isfinite(x))
    throw std::invalid_argument("ElasticModel1DQuadratic deformation variable must be finite");
}

double ElasticModel1DQuadratic::compute_psi(std::span<const double> param, double x) const
{
  validateParameters(param);
  validateX(x);
  return 0.5 * coeff_ * param[0] * x * x;
}

double ElasticModel1DQuadratic::compute_dpsi_dx(std::span<const double> param, double x) const
{
  validateParameters(param);
  validateX(x);
  return coeff_ * param[0] * x;
}

double ElasticModel1DQuadratic::compute_d2psi_dx2(std::span<const double> param, double x) const
{
  validateParameters(param);
  validateX(x);
  (void)x;
  return coeff_ * param[0];
}

double ElasticModel1DQuadratic::compute_dpsi_dparam(
  std::span<const double> param, int i, double x) const
{
  validateParameters(param);
  validateParameterIndex(i);
  validateX(x);
  return 0.5 * coeff_ * x * x;
}

double ElasticModel1DQuadratic::compute_d2psi_dx_dparam(
  std::span<const double> param, int i, double x) const
{
  validateParameters(param);
  validateParameterIndex(i);
  validateX(x);
  return coeff_ * x;
}

double ElasticModel1DQuadratic::compute_d2psi_dparam2(
  std::span<const double> param, int i, int j, double x) const
{
  validateParameters(param);
  validateParameterIndex(i);
  validateParameterIndex(j);
  validateX(x);
  (void)x;
  return 0.0;
}

}  // namespace pgo::SolidDeformationModel
