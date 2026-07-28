/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/model/elasticModelDefinition.h"

#include "material/elastic/elasticModel3DDeformationGradient.h"
#include "EigenSupport.h"

namespace pgo
{
namespace SolidDeformationModel
{
class ElasticModelHillTypeMaterial : public ElasticModel3DDeformationGradient
{
public:
  ElasticModelHillTypeMaterial(double shapeParam, double maximalContractionForce,
    double optimalLengthRatio, const EigenSupport::V3d &fiberDirection);
  virtual ~ElasticModelHillTypeMaterial() {}

  double compute_psi(std::span<const double> param,
    const SpectralState &state) const override;
  EigenSupport::M3d compute_P(std::span<const double> param,
    const SpectralState &state) const override;
  EigenSupport::M9d compute_dPdF(std::span<const double> param,
    const SpectralState &state) const override;

  virtual int getNumParameters() const override { return 1; }
  void compute_dpsi_dparams(std::span<const double> param,
    const SpectralState &state,
    EigenSupport::RefVecXd dpsiDparam) const override;
  void compute_d2psi_dparams2(std::span<const double> param,
    const SpectralState &state,
    EigenSupport::RefMatXd d2psiDparam2) const override;
  void compute_dP_dparams(std::span<const double> param,
    const SpectralState &state,
    EigenSupport::RefMatXd dP_dparam) const override;

  EigenSupport::V3d primaryAxis() const
  {
    return fiberDirection;
  }

protected:
  struct LengthResult
  {
    double value = 0.0;
    EigenSupport::V3d deformedFiber = EigenSupport::V3d::Zero();
  };

  LengthResult compute_length(const EigenSupport::M3d &F) const;
  EigenSupport::M3d compute_dldF(const EigenSupport::V3d &Fd) const;
  EigenSupport::M9d compute_d2ldF2(const EigenSupport::V3d &Fd) const;

  double gamma;
  double maxf;
  double lo;
  EigenSupport::V3d fiberDirection;

  double sqrt_gamma;
  double erf_sqrt_gamma;
  double sqrt_pi;
  EigenSupport::M9d dFddT_dF;

};

class HillStableNeoDefinition final : public ElasticModelDefinition
{
public:
  std::string_view id() const override { return "hill_stable_neo"; }
  MaterialChannelSchema optimizableChannelSchema() const override;
  MaterialChannelSchema fixedChannelSchema() const override;
  std::unique_ptr<ElasticModel> createModel(std::span<const double>, const MaterialFrame &) const override;
private:
};
class HillStVKDefinition final : public ElasticModelDefinition
{
public:
  std::string_view id() const override { return "hill_stvk"; }
  MaterialChannelSchema optimizableChannelSchema() const override;
  MaterialChannelSchema fixedChannelSchema() const override;
  std::unique_ptr<ElasticModel> createModel(std::span<const double>, const MaterialFrame &) const override;
private:
};
class HillStVKVolumeDefinition final : public ElasticModelDefinition
{
public:
  std::string_view id() const override { return "hill_stvk_vol"; }
  MaterialChannelSchema optimizableChannelSchema() const override;
  MaterialChannelSchema fixedChannelSchema() const override;
  std::unique_ptr<ElasticModel> createModel(std::span<const double>, const MaterialFrame &) const override;
private:
};
}  // namespace SolidDeformationModel
}  // namespace pgo
