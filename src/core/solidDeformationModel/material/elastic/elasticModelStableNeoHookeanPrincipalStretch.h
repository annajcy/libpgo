#pragma once

#include "material/elastic/elasticModel3DIsotropicPrincipalStretch.h"

namespace pgo::SolidDeformationModel
{

// Stable Neo-Hookean written directly as an isotropic principal-stretch
// energy. The public (mu, lambda) convention matches
// ElasticModelStableNeoHookeanMaterial.
class ElasticModelStableNeoHookeanPrincipalStretch final
  : public ElasticModel3DIsotropicPrincipalStretch
{
public:
  ElasticModelStableNeoHookeanPrincipalStretch(double mu, double lambda);

  int getNumParameters() const override { return 0; }

  void setMaterial(double mu, double lambda);

protected:
  double compute_psi_s(
    const double *param,
    const EigenSupport::V3d &s) const override;

  EigenSupport::V3d compute_dpsi_ds(
    const double *param,
    const EigenSupport::V3d &s) const override;

  EigenSupport::M3d compute_d2psi_ds2(
    const double *param,
    const EigenSupport::V3d &s) const override;

private:
  double mu_ = 0.0;
  double lambda_ = 0.0;
  double ratio_ = 0.0;
};

}  // namespace pgo::SolidDeformationModel
