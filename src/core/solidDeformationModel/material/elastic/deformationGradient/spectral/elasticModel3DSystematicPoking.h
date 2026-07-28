#pragma once

#include "material/elastic/deformationGradient/spectral/elasticModel3DValanisLandel.h"

#include <span>
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

}  // namespace pgo::SolidDeformationModel
