#pragma once

#include "material/elastic/elasticModel1D.h"
#include "material/elastic/spline/integratedLinearCurvatureSpline.h"

#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

// One-dimensional C2 spline energy whose parameters are the sampled
// curvatures:
//   param[k] = psi''(knots[k]).
// The anchor value and slope are fixed integration constants.
class ElasticModel1DIntegratedLinearCurvatureSpline : public ElasticModel1D
{
public:
  ElasticModel1DIntegratedLinearCurvatureSpline(
    std::span<const double> knots,
    int anchorIndex,
    double anchorValue,
    double anchorSlope);

  ~ElasticModel1DIntegratedLinearCurvatureSpline() override = default;

  double compute_psi(std::span<const double> param, double x) const override;
  double compute_dpsi_dx(std::span<const double> param, double x) const override;
  double compute_d2psi_dx2(std::span<const double> param, double x) const override;

  int getNumParameters() const override;
  double compute_dpsi_dparam(
    std::span<const double> param, int i, double x) const override;
  double compute_d2psi_dx_dparam(
    std::span<const double> param, int i, double x) const override;
  double compute_d2psi_dparam2(
    std::span<const double> param, int i, int j, double x) const override;

private:
  void validateParameters(std::span<const double> param) const;
  void validateParameterIndex(int i) const;
  static void validateX(double x);

  IntegratedLinearCurvatureSpline spline_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
