#include "material/elastic/spline/integratedLinearCurvatureSpline.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace
{

void advanceIntegratedState(
  double curvature,
  double curvatureSlope,
  double dx,
  double &value,
  double &slope)
{
  const double dx2 = dx * dx;
  value += slope * dx + 0.5 * curvature * dx2 +
    curvatureSlope * dx2 * dx / 6.0;
  slope += curvature * dx + 0.5 * curvatureSlope * dx2;
}

template<class CurvatureAt>
void integrateLinearCurvatureTo(
  std::span<const double> knots,
  int anchorIndex,
  double initialValue,
  double initialSlope,
  CurvatureAt curvatureAt,
  double x,
  double &value,
  double &slope)
{
  value = initialValue;
  slope = initialSlope;

  double position = knots[static_cast<std::size_t>(anchorIndex)];
  if (x > position) {
    int interval = anchorIndex;
    while (position < x &&
      interval < static_cast<int>(knots.size()) - 1) {
      const double intervalEnd =
        knots[static_cast<std::size_t>(interval + 1)];
      const double target = std::min(x, intervalEnd);
      const double intervalWidth =
        intervalEnd - knots[static_cast<std::size_t>(interval)];
      const double curvature = curvatureAt(interval);
      const double curvatureSlope =
        (curvatureAt(interval + 1) - curvature) /
        intervalWidth;
      advanceIntegratedState(
        curvature,
        curvatureSlope,
        target - position,
        value,
        slope);
      position = target;
      ++interval;
    }
    if (position < x) {
      advanceIntegratedState(
        curvatureAt(static_cast<int>(knots.size()) - 1),
        0.0,
        x - position,
        value,
        slope);
    }
  }
  else if (x < position) {
    int interval = anchorIndex - 1;
    while (position > x && interval >= 0) {
      const double intervalStart =
        knots[static_cast<std::size_t>(interval)];
      const double target = std::max(x, intervalStart);
      const double intervalWidth =
        knots[static_cast<std::size_t>(interval + 1)] -
        intervalStart;
      const double curvature = curvatureAt(interval + 1);
      const double curvatureSlope =
        (curvature - curvatureAt(interval)) /
        intervalWidth;
      advanceIntegratedState(
        curvature,
        curvatureSlope,
        target - position,
        value,
        slope);
      position = target;
      --interval;
    }
    if (position > x) {
      advanceIntegratedState(
        curvatureAt(0), 0.0, x - position, value, slope);
    }
  }
}

}  // namespace

IntegratedLinearCurvatureSpline::IntegratedLinearCurvatureSpline(
  std::span<const double> knots,
  int anchorIndex,
  double anchorValue,
  double anchorSlope):
  knots_(knots.begin(), knots.end()),
  anchorIndex_(anchorIndex),
  anchorValue_(anchorValue),
  anchorSlope_(anchorSlope)
{
  if (knots_.size() < 2)
    throw std::invalid_argument(
      "IntegratedLinearCurvatureSpline requires at least two knots.");
  for (double knot : knots_) {
    if (!std::isfinite(knot))
      throw std::invalid_argument(
        "IntegratedLinearCurvatureSpline knots must be finite.");
  }
  for (std::size_t i = 1; i < knots_.size(); ++i) {
    if (!(knots_[i] > knots_[i - 1]))
      throw std::invalid_argument(
        "IntegratedLinearCurvatureSpline knots must be strictly increasing.");
  }
  if (anchorIndex_ < 0 ||
    anchorIndex_ >= static_cast<int>(knots_.size()))
    throw std::out_of_range(
      "IntegratedLinearCurvatureSpline anchor index is out of range.");
  if (!std::isfinite(anchorValue_) || !std::isfinite(anchorSlope_))
    throw std::invalid_argument(
      "IntegratedLinearCurvatureSpline anchor conditions must be finite.");
}

void IntegratedLinearCurvatureSpline::validateEvaluationInputs(
  std::span<const double> curvatureValues,
  double x) const
{
  if (curvatureValues.size() != knots_.size())
    throw std::invalid_argument(
      "IntegratedLinearCurvatureSpline curvature count must match the knot count.");
  for (double curvature : curvatureValues) {
    if (!std::isfinite(curvature))
      throw std::invalid_argument(
        "IntegratedLinearCurvatureSpline curvatures must be finite.");
  }
  validateQuery(x);
}

void IntegratedLinearCurvatureSpline::validateQuery(double x) const
{
  if (!std::isfinite(x))
    throw std::invalid_argument(
      "IntegratedLinearCurvatureSpline query must be finite.");
}

void IntegratedLinearCurvatureSpline::validateCurvatureIndex(
  int curvatureIndex) const
{
  if (curvatureIndex < 0 ||
    curvatureIndex >= static_cast<int>(knots_.size()))
    throw std::out_of_range(
      "IntegratedLinearCurvatureSpline curvature index is out of range.");
}

void IntegratedLinearCurvatureSpline::integrateTo(
  std::span<const double> curvatureValues,
  double x,
  double &value,
  double &slope) const
{
  integrateLinearCurvatureTo(
    knots_,
    anchorIndex_,
    anchorValue_,
    anchorSlope_,
    [&](int index) {
      return curvatureValues[static_cast<std::size_t>(index)];
    },
    x,
    value,
    slope);
}

double IntegratedLinearCurvatureSpline::y(
  std::span<const double> curvatureValues,
  double x) const
{
  validateEvaluationInputs(curvatureValues, x);
  double value = 0.0;
  double slope = 0.0;
  integrateTo(curvatureValues, x, value, slope);
  return value;
}

double IntegratedLinearCurvatureSpline::dy_dx(
  std::span<const double> curvatureValues,
  double x) const
{
  validateEvaluationInputs(curvatureValues, x);
  double value = 0.0;
  double slope = 0.0;
  integrateTo(curvatureValues, x, value, slope);
  return slope;
}

double IntegratedLinearCurvatureSpline::d2y_dx2(
  std::span<const double> curvatureValues,
  double x) const
{
  validateEvaluationInputs(curvatureValues, x);

  if (x <= knots_.front())
    return curvatureValues.front();
  if (x >= knots_.back())
    return curvatureValues.back();

  const auto upper = std::lower_bound(knots_.begin(), knots_.end(), x);
  if (upper != knots_.end() && *upper == x) {
    return curvatureValues[static_cast<std::size_t>(upper - knots_.begin())];
  }

  const std::size_t right =
    static_cast<std::size_t>(upper - knots_.begin());
  const std::size_t left = right - 1;
  const double t =
    (x - knots_[left]) / (knots_[right] - knots_[left]);
  return (1.0 - t) * curvatureValues[left] +
    t * curvatureValues[right];
}

double IntegratedLinearCurvatureSpline::dy_dcurvature(
  int curvatureIndex,
  double x) const
{
  validateCurvatureIndex(curvatureIndex);
  validateQuery(x);

  double value = 0.0;
  double slope = 0.0;
  integrateLinearCurvatureTo(
    knots_,
    anchorIndex_,
    0.0,
    0.0,
    [&](int index) {
      return index == curvatureIndex ? 1.0 : 0.0;
    },
    x,
    value,
    slope);
  return value;
}

double IntegratedLinearCurvatureSpline::d2y_dx_dcurvature(
  int curvatureIndex,
  double x) const
{
  validateCurvatureIndex(curvatureIndex);
  validateQuery(x);

  double value = 0.0;
  double slope = 0.0;
  integrateLinearCurvatureTo(
    knots_,
    anchorIndex_,
    0.0,
    0.0,
    [&](int index) {
      return index == curvatureIndex ? 1.0 : 0.0;
    },
    x,
    value,
    slope);
  return slope;
}

}  // namespace pgo::SolidDeformationModel
