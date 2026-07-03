#pragma once

#include "material/elastic/elasticModel.h"

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{

// 1D cubic-spline elastic energy that interpolates a user-provided
// stress-strain curve.  Used with 1D / tet-occupation constraints.
class ElasticModel1DCubicSplineImpl;

class ElasticModel1DCubicSpline : public ElasticModel
{
public:
  ElasticModel1DCubicSpline(double coeff, int nPoints, double xLeft, double xRight);

  virtual ~ElasticModel1DCubicSpline();

  double compute_psi(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3]) const;
  void compute_P(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double P[9]) const;
  void compute_dPdF(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double dPdF[81]) const;

  int getNumParameters() const override;
  double compute_dpsi_dparam(const double *param, int i, const double F[9],
    const double U[9], const double V[9], const double S[3]) const;
  double compute_d2psi_dparam2(const double *param, int i, int j,
    const double F[9], const double U[9], const double V[9], const double S[3]) const;
  void compute_dP_dparam(const double *param, int i, const double F[9],
    const double U[9], const double V[9], const double S[3], double *ret) const;

private:
  std::unique_ptr<ElasticModel1DCubicSplineImpl> impl_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
