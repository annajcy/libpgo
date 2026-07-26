#pragma once

#include "material/elastic/elasticModel3DDeformationGradient.h"
#include "material/elastic/deformationGradient/spectral/isotropicSpectralTangent.h"

namespace pgo::SolidDeformationModel
{

class ElasticModel3DIsotropicPrincipalStretch
  : public ElasticModel3DDeformationGradient
{
public:
  ~ElasticModel3DIsotropicPrincipalStretch() override = default;

  double compute_psi(
    const double *param,
    const double F[9],
    const double U[9],
    const double V[9],
    const double S[3]) const final;

  void compute_P(
    const double *param,
    const double F[9],
    const double U[9],
    const double V[9],
    const double S[3],
    double P[9]) const final;

  void compute_dPdF(
    const double *param,
    const double F[9],
    const double U[9],
    const double V[9],
    const double S[3],
    double dPdF[81]) const final;

  void compute_dPdF_psd(
    const double *param,
    const double F[9],
    const double U[9],
    const double V[9],
    const double S[3],
    double dPdF[81]) const final;

protected:
  virtual double compute_psi_s(
    const double *param,
    const EigenSupport::V3d &s) const = 0;

  virtual EigenSupport::V3d compute_dpsi_ds(
    const double *param,
    const EigenSupport::V3d &s) const = 0;

  virtual EigenSupport::M3d compute_d2psi_ds2(
    const double *param,
    const EigenSupport::V3d &s) const = 0;
};

}  // namespace pgo::SolidDeformationModel
