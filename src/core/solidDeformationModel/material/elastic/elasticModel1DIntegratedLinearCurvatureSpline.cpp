#include "material/elastic/elasticModel1DIntegratedLinearCurvatureSpline.h"

#include <cmath>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{

ElasticModel1DIntegratedLinearCurvatureSpline::
  ElasticModel1DIntegratedLinearCurvatureSpline(
    std::span<const double> knots,
    int anchorIndex,
    double anchorValue,
    double anchorSlope):
  spline_(knots, anchorIndex, anchorValue, anchorSlope)
{
}

void ElasticModel1DIntegratedLinearCurvatureSpline::validateParameters(
  std::span<const double> param) const
{
  if (param.size() != static_cast<std::size_t>(getNumParameters()))
    throw std::invalid_argument(
      "ElasticModel1DIntegratedLinearCurvatureSpline parameter count "
      "must match the knot count");
  for (double value : param) {
    if (!std::isfinite(value))
      throw std::invalid_argument(
        "ElasticModel1DIntegratedLinearCurvatureSpline parameters "
        "must be finite");
  }
}

void ElasticModel1DIntegratedLinearCurvatureSpline::
  validateParameterIndex(int i) const
{
  if (i < 0 || i >= getNumParameters())
    throw std::out_of_range(
      "ElasticModel1DIntegratedLinearCurvatureSpline parameter "
      "index is out of range");
}

void ElasticModel1DIntegratedLinearCurvatureSpline::validateX(double x)
{
  if (!std::isfinite(x))
    throw std::invalid_argument(
      "ElasticModel1DIntegratedLinearCurvatureSpline deformation "
      "variable must be finite");
}

double ElasticModel1DIntegratedLinearCurvatureSpline::compute_psi(
  std::span<const double> param, double x) const
{
  return spline_.y(param, x);
}

double ElasticModel1DIntegratedLinearCurvatureSpline::compute_dpsi_dx(
  std::span<const double> param, double x) const
{
  return spline_.dy_dx(param, x);
}

double ElasticModel1DIntegratedLinearCurvatureSpline::compute_d2psi_dx2(
  std::span<const double> param, double x) const
{
  return spline_.d2y_dx2(param, x);
}

int ElasticModel1DIntegratedLinearCurvatureSpline::getNumParameters() const
{
  return spline_.numKnots();
}

double ElasticModel1DIntegratedLinearCurvatureSpline::
  compute_dpsi_dparam(
    std::span<const double> param, int i, double x) const
{
  validateParameters(param);
  return spline_.dy_dcurvature(i, x);
}

double ElasticModel1DIntegratedLinearCurvatureSpline::
  compute_d2psi_dx_dparam(
    std::span<const double> param, int i, double x) const
{
  validateParameters(param);
  return spline_.d2y_dx_dcurvature(i, x);
}

double ElasticModel1DIntegratedLinearCurvatureSpline::
  compute_d2psi_dparam2(
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
