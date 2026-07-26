#pragma once

#include "material/elastic/elasticModel3DDeformationGradient.h"
#include "EigenDef.h"

namespace pgo
{
namespace SolidDeformationModel
{
class ElasticModel3DMooneyRivlin : public ElasticModel3DDeformationGradient
{
public:
  ElasticModel3DMooneyRivlin(double mu01, double mu10, double v1);
  virtual ~ElasticModel3DMooneyRivlin() {}

  int getNumParameters() const override { return 0; }

  double compute_psi(std::span<const double> param,
    const SpectralState &state) const override;
  EigenSupport::M3d compute_P(std::span<const double> param,
    const SpectralState &state) const override;
  EigenSupport::M9d compute_dPdF(std::span<const double> param,
    const SpectralState &state) const override;

protected:
  double mu01_;
  double mu10_;
  double v1_;
};

class MooneyRivlinConfig final : public ElasticModelConfig
{
public:
  std::string_view id() const override { return "mooney_rivlin"; }
  std::span<const std::string_view> parameterChannelNames() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  void initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<ElasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
