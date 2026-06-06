#pragma once

#include "elastic/elasticModel.h"

namespace pgo
{
namespace SolidDeformationModel
{

// 1D quadratic elastic energy: psi = c * (lambda - 1)^2, where lambda = F[0] / F[1].
// Simplified deformation model that operates on a single stretch axis — used with
// 1D / tet-occupation constraints.
class ElasticModel1DQuadratic : public ElasticModel
{
public:
  explicit ElasticModel1DQuadratic(double c):
    coeff(c)
  {
  }

  virtual ~ElasticModel1DQuadratic() {}

  double compute_psi(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3]) const;
  void compute_P(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double P[9]) const;
  void compute_dPdF(const double *param, const double F[9],
    const double U[9], const double V[9], const double S[3], double dPdF[81]) const;

  int getNumParameters() const override { return 1; }
  double compute_dpsi_dparam(const double *param, int i, const double F[9],
    const double U[9], const double V[9], const double S[3]) const;
  double compute_d2psi_dparam2(const double *param, int i, int j,
    const double F[9], const double U[9], const double V[9], const double S[3]) const;
  void compute_dP_dparam(const double *param, int i, const double F[9],
    const double U[9], const double V[9], const double S[3], double *ret) const;

protected:
  double coeff;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
