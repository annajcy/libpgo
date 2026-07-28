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
  if (!std::isfinite(x))
    throw std::invalid_argument(
      "IntegratedLinearCurvatureSpline query must be finite.");
}

void IntegratedLinearCurvatureSpline::integrateTo(
  std::span<const double> curvatureValues,
  double x,
  double &value,
  double &slope) const
{
  value = anchorValue_;
  slope = anchorSlope_;

  double position = knots_[static_cast<std::size_t>(anchorIndex_)];
  if (x > position) {
    int interval = anchorIndex_;
    while (position < x &&
      interval < static_cast<int>(knots_.size()) - 1) {
      const double intervalEnd =
        knots_[static_cast<std::size_t>(interval + 1)];
      const double target = std::min(x, intervalEnd);
      const double intervalWidth =
        intervalEnd - knots_[static_cast<std::size_t>(interval)];
      const double curvatureSlope =
        (curvatureValues[static_cast<std::size_t>(interval + 1)] -
          curvatureValues[static_cast<std::size_t>(interval)]) /
        intervalWidth;
      advanceIntegratedState(
        curvatureValues[static_cast<std::size_t>(interval)],
        curvatureSlope,
        target - position,
        value,
        slope);
      position = target;
      ++interval;
    }
    if (position < x) {
      advanceIntegratedState(
        curvatureValues.back(), 0.0, x - position, value, slope);
    }
  }
  else if (x < position) {
    int interval = anchorIndex_ - 1;
    while (position > x && interval >= 0) {
      const double intervalStart =
        knots_[static_cast<std::size_t>(interval)];
      const double target = std::max(x, intervalStart);
      const double intervalWidth =
        knots_[static_cast<std::size_t>(interval + 1)] - intervalStart;
      const double curvatureSlope =
        (curvatureValues[static_cast<std::size_t>(interval + 1)] -
          curvatureValues[static_cast<std::size_t>(interval)]) /
        intervalWidth;
      advanceIntegratedState(
        curvatureValues[static_cast<std::size_t>(interval + 1)],
        curvatureSlope,
        target - position,
        value,
        slope);
      position = target;
      --interval;
    }
    if (position > x) {
      advanceIntegratedState(
        curvatureValues.front(), 0.0, x - position, value, slope);
    }
  }
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

}  // namespace pgo::SolidDeformationModel
