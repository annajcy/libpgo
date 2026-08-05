#pragma once

#include "material/elastic/elasticModelDefinition.h"

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

class MooneyRivlinDefinition final : public ElasticModelDefinition
{
public:
  std::string_view id() const override { return "mooney_rivlin"; }
  int numOptimizableChannels() const override;
  int numFixedChannels() const override;
  std::unique_ptr<ElasticModel> createModel(std::span<const double>, const MaterialFrame &) const override;
private:
};
}  // namespace SolidDeformationModel
}  // namespace pgo
