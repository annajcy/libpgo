#pragma once

#include "material/elastic/deformationGradient/spectral/elasticModel3DIsotropicPrincipalStretch.h"
#include "material/elastic/elasticModel1D.h"

#include <memory>
#include <span>

namespace pgo::SolidDeformationModel
{

// General compressible Valanis-Landel energy:
//   psi(s) = sum_i f(s_i)
//          + sum_{i<j} g(s_i s_j)
//          + h(s_0 s_1 s_2).
//
// The external parameter vector is laid out as [f parameters,
// g parameters, h parameters].
class ElasticModel3DValanisLandel : public ElasticModel3DIsotropicPrincipalStretch
{
public:
  ElasticModel3DValanisLandel(
    std::shared_ptr<const ElasticModel1D> f,
    std::shared_ptr<const ElasticModel1D> g,
    std::shared_ptr<const ElasticModel1D> h);

  ~ElasticModel3DValanisLandel() override = default;

  int getNumParameters() const override
  {
    return fParameterCount_ + gParameterCount_ + hParameterCount_;
  }

  void compute_dpsi_dparams(
    std::span<const double> param,
    const SpectralState &state,
    EigenSupport::RefVecXd dpsiDparam) const override;

  void compute_d2psi_dparams2(
    std::span<const double> param,
    const SpectralState &state,
    EigenSupport::RefMatXd d2psiDparam2) const override;

  void compute_dP_dparams(
    std::span<const double> param,
    const SpectralState &state,
    EigenSupport::RefMatXd dP_dparam) const override;

protected:
  double compute_psi_s(
    std::span<const double> param,
    const EigenSupport::V3d &s) const override;

  EigenSupport::V3d compute_dpsi_ds(
    std::span<const double> param,
    const EigenSupport::V3d &s) const override;

  EigenSupport::M3d compute_d2psi_ds2(
    std::span<const double> param,
    const EigenSupport::V3d &s) const override;

private:
  struct ParameterSpans
  {
    std::span<const double> f;
    std::span<const double> g;
    std::span<const double> h;
  };

  ParameterSpans splitParameters(
    std::span<const double> param) const;

  std::shared_ptr<const ElasticModel1D> f_;
  std::shared_ptr<const ElasticModel1D> g_;
  std::shared_ptr<const ElasticModel1D> h_;
  int fParameterCount_ = 0;
  int gParameterCount_ = 0;
  int hParameterCount_ = 0;
};

}  // namespace pgo::SolidDeformationModel
