#include "material/elastic/elasticModel1DZero.h"

#include <cmath>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{

void ElasticModel1DZero::validateInputs(
  std::span<const double> param, double x)
{
  if (!param.empty())
    throw std::invalid_argument(
      "ElasticModel1DZero expects no material parameters");
  if (!std::isfinite(x))
    throw std::invalid_argument(
      "ElasticModel1DZero deformation variable must be finite");
}

void ElasticModel1DZero::throwParameterIndexOutOfRange()
{
  throw std::out_of_range(
    "ElasticModel1DZero has no material parameters");
}

double ElasticModel1DZero::compute_psi(
  std::span<const double> param, double x) const
{
  validateInputs(param, x);
  return 0.0;
}

double ElasticModel1DZero::compute_dpsi_dx(
  std::span<const double> param, double x) const
{
  validateInputs(param, x);
  return 0.0;
}

double ElasticModel1DZero::compute_d2psi_dx2(
  std::span<const double> param, double x) const
{
  validateInputs(param, x);
  return 0.0;
}

double ElasticModel1DZero::compute_dpsi_dparam(
  std::span<const double> param, int i, double x) const
{
  validateInputs(param, x);
  (void)i;
  throwParameterIndexOutOfRange();
}

double ElasticModel1DZero::compute_d2psi_dx_dparam(
  std::span<const double> param, int i, double x) const
{
  validateInputs(param, x);
  (void)i;
  throwParameterIndexOutOfRange();
}

double ElasticModel1DZero::compute_d2psi_dparam2(
  std::span<const double> param, int i, int j, double x) const
{
  validateInputs(param, x);
  (void)i;
  (void)j;
  throwParameterIndexOutOfRange();
}

}  // namespace pgo::SolidDeformationModel
