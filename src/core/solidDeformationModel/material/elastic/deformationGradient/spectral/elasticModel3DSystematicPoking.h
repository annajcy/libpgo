#pragma once

#include "material/elastic/deformationGradient/spectral/elasticModel3DValanisLandel.h"
#include "material/model/elasticModelDefinition.h"

#include <span>
#include <string_view>
#include <vector>

namespace pgo::SolidDeformationModel
{

// Samples h''(J) for
//   h(J; lambda) = 1/2 * lambda * log(J)^2
// at positive volume knots.
std::vector<double> sampleLogSquaredVolumeCurvatures(
  std::span<const double> volumeKnots,
  double lambda);

// The animation-ready generalized Neo-Hookean material used by Systematic
// Poking:
//   f: integrated-linear-curvature spline with f(1) = f'(1) = 0,
//   g: zero,
//   h: a fixed integrated-linear-curvature spline shape sampled from
//      1/2 * log(J)^2 and scaled by lambda.
//
// Parameters are laid out as
//   [f''(stretchKnots[0]), ..., f''(stretchKnots[n - 1]), lambda].
class ElasticModel3DSystematicPoking final : public ElasticModel3DValanisLandel
{
public:
  ElasticModel3DSystematicPoking(
    std::span<const double> stretchKnots,
    int stretchRestKnotIndex,
    std::span<const double> volumeKnots,
    int volumeRestKnotIndex);
};

/// Immutable structural definition for the paper's Systematic Poking
/// material.
///
/// Knot locations and rest-knot indices determine the constitutive basis and
/// are therefore stored once in the definition. Per-element material fields
/// contain only the optimizable physical channels
///
///   [f''(stretchKnots[0]), ..., f''(stretchKnots[n - 1]), lambda].
class SystematicPokingDefinition final : public ElasticModelDefinition
{
public:
  SystematicPokingDefinition(
    std::span<const double> stretchKnots,
    int stretchRestKnotIndex,
    std::span<const double> volumeKnots,
    int volumeRestKnotIndex);

  std::string_view id() const override
  {
    return "systematic_poking";
  }

  MaterialChannelSchema fixedChannelSchema() const override;
  MaterialChannelSchema optimizableChannelSchema() const override;
  std::unique_ptr<ElasticModel> createModel(
    std::span<const double> fixedChannels,
    const MaterialFrame &materialFrame) const override;

  std::span<const double> stretchKnots() const
  {
    return stretchKnots_;
  }
  int stretchRestKnotIndex() const
  {
    return stretchRestKnotIndex_;
  }
  std::span<const double> volumeKnots() const
  {
    return volumeKnots_;
  }
  int volumeRestKnotIndex() const
  {
    return volumeRestKnotIndex_;
  }

private:
  std::vector<double> stretchKnots_;
  int stretchRestKnotIndex_ = 0;
  std::vector<double> volumeKnots_;
  int volumeRestKnotIndex_ = 0;
  MaterialChannelSchema optimizableChannelSchema_;
};

}  // namespace pgo::SolidDeformationModel
