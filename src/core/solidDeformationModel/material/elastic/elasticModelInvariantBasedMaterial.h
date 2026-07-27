/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/elastic/elasticModel3DDeformationGradient.h"

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{
class InvariantBasedMaterial;

class ElasticModelInvariantBasedMaterial : public ElasticModel3DDeformationGradient
{
public:
  explicit ElasticModelInvariantBasedMaterial(std::unique_ptr<InvariantBasedMaterial> invMat);
  ~ElasticModelInvariantBasedMaterial() override = default;

  int getNumParameters() const override { return 0; }

  double compute_psi(std::span<const double> param,
    const SpectralState &state) const override;
  EigenSupport::M3d compute_P(std::span<const double> param,
    const SpectralState &state) const override;
  EigenSupport::M9d compute_dPdF(std::span<const double> param,
    const SpectralState &state) const override;
  EigenSupport::M9d compute_dPdF_psd(std::span<const double> param,
    const SpectralState &state) const override;

  const InvariantBasedMaterial *getInvariantBasedMaterial() const { return invariantBasedMaterial_.get(); }

protected:
  std::unique_ptr<InvariantBasedMaterial> invariantBasedMaterial_;

private:
  EigenSupport::M9d compute_dPdF_impl(std::span<const double> param,
    const SpectralState &state, bool project) const;
};

class InvariantStVKDefinition final : public ElasticModelDefinition
{
public:
  std::string_view id() const override { return "inv_stvk"; }
  MaterialChannelSchema optimizableChannelSchema() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  MaterialChannelSchema fixedChannelSchema() const override;
  std::unique_ptr<ElasticModel> createModelFromFixed(std::span<const double>, const MaterialFrame &) const override;
private:
};
}  // namespace SolidDeformationModel
}  // namespace pgo
