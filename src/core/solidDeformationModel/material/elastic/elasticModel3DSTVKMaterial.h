#pragma once

#include "material/elastic/elasticModel3DDeformationGradient.h"
#include "EigenDef.h"

#include <array>

namespace pgo
{
namespace SolidDeformationModel
{
class ElasticModel3DSTVKMaterial : public ElasticModel3DDeformationGradient
{
public:
  ElasticModel3DSTVKMaterial(double _mu, double _lambda);
  virtual ~ElasticModel3DSTVKMaterial();

  int getNumParameters() const override { return 0; }
  void setMaterialParameters(double _mu, double _lambda)   {     mu = _mu;     lambda = _lambda;   }

  double compute_psi(std::span<const double> param,
    const SpectralState &state) const override;
  EigenSupport::M3d compute_P(std::span<const double> param,
    const SpectralState &state) const override;         // d||P||/dP  d||Q||/dQ
  EigenSupport::M9d compute_dPdF(std::span<const double> param,
    const SpectralState &state) const override;  // d2||P||/dP2  d2||Q||/dQ2

protected:
  struct LowerInvarianceResult
  {
    EigenSupport::V3d invariants;
    EigenSupport::M3d rotation;
    EigenSupport::V3d signedStretches;
    EigenSupport::M3d U;
    EigenSupport::M3d V;
  };

  LowerInvarianceResult computeLowerInvariance(const EigenSupport::M3d &F) const;

  double mu, lambda;
  std::array<EigenSupport::M3d, 3> C;
};

class StVKConfig final : public ElasticModelConfig
{
public:
  std::string_view id() const override { return "stvk"; }
  std::span<const std::string_view> parameterChannelNames() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  void initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<ElasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
