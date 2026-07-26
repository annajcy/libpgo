#pragma once

#include "material/elastic/elasticModel1D.h"

#include <memory>
#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

class ElasticModel1DCubicSplineImpl;

// One-dimensional spline energy:
//   psi(x; p) = coeff * spline(x; p).
// The parameter vector contains the spline knot values.
class ElasticModel1DCubicSpline : public ElasticModel1D
{
public:
  ElasticModel1DCubicSpline(double coeff, int nPoints, double xLeft, double xRight);
  ~ElasticModel1DCubicSpline() override;

  double compute_psi(std::span<const double> param, double x) const override;
  double compute_dpsi_dx(std::span<const double> param, double x) const override;
  double compute_d2psi_dx2(std::span<const double> param, double x) const override;

  int getNumParameters() const override;
  double compute_dpsi_dparam(std::span<const double> param, int i, double x) const override;
  double compute_d2psi_dx_dparam(std::span<const double> param, int i, double x) const override;
  double compute_d2psi_dparam2(
    std::span<const double> param, int i, int j, double x) const override;

private:
  std::unique_ptr<ElasticModel1DCubicSplineImpl> impl_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
