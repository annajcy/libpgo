#pragma once

#include <span>
#include <vector>

namespace pgo::SolidDeformationModel
{

/// A C2 scalar spline defined by linearly interpolated second derivatives.
///
/// curvatureValues[k] is y''(knots[k]). Outside the knot range, y'' is held
/// constant at the nearest endpoint value, so y extrapolates quadratically.
/// The anchor conditions fix the two integration constants at one knot.
class IntegratedLinearCurvatureSpline
{
public:
  IntegratedLinearCurvatureSpline(
    std::span<const double> knots,
    int anchorIndex,
    double anchorValue,
    double anchorSlope);

  int numKnots() const { return static_cast<int>(knots_.size()); }
  int anchorIndex() const { return anchorIndex_; }
  std::span<const double> knots() const { return knots_; }

  double y(
    std::span<const double> curvatureValues,
    double x) const;

  double dy_dx(
    std::span<const double> curvatureValues,
    double x) const;

  double d2y_dx2(
    std::span<const double> curvatureValues,
    double x) const;

private:
  void validateEvaluationInputs(
    std::span<const double> curvatureValues,
    double x) const;

  void integrateTo(
    std::span<const double> curvatureValues,
    double x,
    double &value,
    double &slope) const;

  std::vector<double> knots_;
  int anchorIndex_ = 0;
  double anchorValue_ = 0.0;
  double anchorSlope_ = 0.0;
};

}  // namespace pgo::SolidDeformationModel
