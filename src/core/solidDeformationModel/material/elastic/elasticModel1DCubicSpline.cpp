#include "material/elastic/elasticModel1DCubicSpline.h"

#include "naturalCubicSplineDerivatives.h"

#include <cmath>
#include <memory>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace ES = pgo::EigenSupport;

class ElasticModel1DCubicSplineImpl
{
public:
  std::shared_ptr<NonlinearOptimization::NaturalCubicSpline2DWithParameterDerivatives> spline;
  ES::VXd xValues;
  double coeff = 0.0;
};

namespace
{
void validateParameters(std::span<const double> param, int expectedSize)
{
  if (param.size() != static_cast<std::size_t>(expectedSize))
    throw std::invalid_argument("ElasticModel1DCubicSpline parameter count does not match the knot count");
  for (double value : param) {
    if (!std::isfinite(value))
      throw std::invalid_argument("ElasticModel1DCubicSpline parameters must be finite");
  }
}

void validateParameterIndex(int i, int parameterCount)
{
  if (i < 0 || i >= parameterCount)
    throw std::out_of_range("ElasticModel1DCubicSpline parameter index is out of range");
}

void validateX(double x)
{
  if (!std::isfinite(x))
    throw std::invalid_argument("ElasticModel1DCubicSpline deformation variable must be finite");
}

ES::Mp<const ES::VXd> mapParameters(std::span<const double> param)
{
  return ES::Mp<const ES::VXd>(param.data(), static_cast<Eigen::Index>(param.size()));
}
}  // namespace

ElasticModel1DCubicSpline::ElasticModel1DCubicSpline(
  double coeff, int nPoints, double xLeft, double xRight)
{
  if (!std::isfinite(coeff) || !std::isfinite(xLeft) || !std::isfinite(xRight))
    throw std::invalid_argument("ElasticModel1DCubicSpline constructor arguments must be finite");
  if (nPoints < 2)
    throw std::invalid_argument("ElasticModel1DCubicSpline requires at least two knots");
  if (!(xRight > xLeft))
    throw std::invalid_argument("ElasticModel1DCubicSpline requires xRight > xLeft");

  impl_ = std::make_unique<ElasticModel1DCubicSplineImpl>();
  impl_->xValues.resize(nPoints);
  const double deltaX = (xRight - xLeft) / static_cast<double>(nPoints - 1);
  for (int i = 0; i < nPoints; ++i)
    impl_->xValues[i] = xLeft + static_cast<double>(i) * deltaX;

  impl_->spline = std::make_shared<NonlinearOptimization::NaturalCubicSpline2DWithParameterDerivatives>(
    impl_->xValues);
  impl_->coeff = coeff;
}

ElasticModel1DCubicSpline::~ElasticModel1DCubicSpline() = default;

double ElasticModel1DCubicSpline::compute_psi(std::span<const double> param, double x) const
{
  validateParameters(param, getNumParameters());
  validateX(x);
  return impl_->coeff * impl_->spline->y(x, mapParameters(param));
}

double ElasticModel1DCubicSpline::compute_dpsi_dx(std::span<const double> param, double x) const
{
  validateParameters(param, getNumParameters());
  validateX(x);
  return impl_->coeff * impl_->spline->dy_dx(x, mapParameters(param));
}

double ElasticModel1DCubicSpline::compute_d2psi_dx2(std::span<const double> param, double x) const
{
  validateParameters(param, getNumParameters());
  validateX(x);
  return impl_->coeff * impl_->spline->d2y_dx2(x, mapParameters(param));
}

int ElasticModel1DCubicSpline::getNumParameters() const
{
  return static_cast<int>(impl_->xValues.size());
}

double ElasticModel1DCubicSpline::compute_dpsi_dparam(
  std::span<const double> param, int i, double x) const
{
  validateParameters(param, getNumParameters());
  validateParameterIndex(i, getNumParameters());
  validateX(x);
  return impl_->coeff * impl_->spline->dy_dparam(x, i);
}

double ElasticModel1DCubicSpline::compute_d2psi_dx_dparam(
  std::span<const double> param, int i, double x) const
{
  validateParameters(param, getNumParameters());
  validateParameterIndex(i, getNumParameters());
  validateX(x);
  return impl_->coeff * impl_->spline->d2y_dparam_dx(x, i);
}

double ElasticModel1DCubicSpline::compute_d2psi_dparam2(
  std::span<const double> param, int i, int j, double x) const
{
  validateParameters(param, getNumParameters());
  validateParameterIndex(i, getNumParameters());
  validateParameterIndex(j, getNumParameters());
  validateX(x);
  // The spline is affine in its knot-value parameters.
  return impl_->coeff * impl_->spline->d2y_dparam2(x, i, j);
}

}  // namespace pgo::SolidDeformationModel
