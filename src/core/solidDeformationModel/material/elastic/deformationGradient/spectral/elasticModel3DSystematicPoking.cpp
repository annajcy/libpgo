#include "material/elastic/deformationGradient/spectral/elasticModel3DSystematicPoking.h"

#include "material/elastic/elasticModel1DFixedParameters.h"
#include "material/elastic/elasticModel1DIntegratedLinearCurvatureSpline.h"
#include "material/elastic/elasticModel1DScaled.h"
#include "material/elastic/elasticModel1DZero.h"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pgo::SolidDeformationModel
{
namespace
{

void validateRestKnot(
  std::span<const double> knots,
  int restKnotIndex,
  const char *coordinateName)
{
  if (restKnotIndex < 0 ||
    restKnotIndex >= static_cast<int>(knots.size()))
    throw std::out_of_range(
      std::string("ElasticModel3DSystematicPoking ") +
      coordinateName + " rest knot index is out of range");
  if (knots[static_cast<std::size_t>(restKnotIndex)] != 1.0)
    throw std::invalid_argument(
      std::string("ElasticModel3DSystematicPoking ") +
      coordinateName + " rest knot must equal one");
}

std::shared_ptr<const ElasticModel1D>
makeSystematicPokingStretchModel(
  std::span<const double> stretchKnots,
  int stretchRestKnotIndex)
{
  validateRestKnot(
    stretchKnots, stretchRestKnotIndex, "stretch");

  return std::make_shared<
    ElasticModel1DIntegratedLinearCurvatureSpline>(
    stretchKnots,
    stretchRestKnotIndex,
    0.0,
    0.0);
}

std::shared_ptr<const ElasticModel1D>
makeSystematicPokingVolumeModel(
  std::span<const double> volumeKnots,
  int volumeRestKnotIndex)
{
  validateRestKnot(
    volumeKnots, volumeRestKnotIndex, "volume");
  auto spline = std::make_shared<
    ElasticModel1DIntegratedLinearCurvatureSpline>(
    volumeKnots,
    volumeRestKnotIndex,
    0.0,
    0.0);
  auto fixedShape =
    std::make_shared<ElasticModel1DFixedParameters>(
      std::move(spline),
      sampleLogSquaredVolumeCurvatures(
        volumeKnots, 1.0));
  return std::make_shared<ElasticModel1DScaled>(
    std::move(fixedShape));
}

MaterialChannelSchema makeSystematicPokingOptimizableChannelSchema(
  std::size_t stretchKnotCount)
{
  std::vector<std::string> names;
  names.reserve(stretchKnotCount + 1);
  for (std::size_t i = 0; i < stretchKnotCount; ++i)
    names.emplace_back("f_dd_" + std::to_string(i));
  names.emplace_back("lambda");
  return MaterialChannelSchema(std::move(names));
}

}  // namespace

std::vector<double> sampleLogSquaredVolumeCurvatures(
  std::span<const double> volumeKnots,
  double lambda)
{
  if (!std::isfinite(lambda))
    throw std::invalid_argument(
      "sampleLogSquaredVolumeCurvatures lambda must be finite");

  std::vector<double> curvatures;
  curvatures.reserve(volumeKnots.size());
  for (double J : volumeKnots) {
    if (!std::isfinite(J) || !(J > 0.0))
      throw std::domain_error(
        "sampleLogSquaredVolumeCurvatures knots must be "
        "finite and positive");

    double curvature =
      lambda * (1.0 - std::log(J));
    curvature /= J;
    curvature /= J;
    if (!std::isfinite(curvature))
      throw std::overflow_error(
        "sampleLogSquaredVolumeCurvatures result is not finite");
    curvatures.push_back(curvature);
  }
  return curvatures;
}

ElasticModel3DSystematicPoking::ElasticModel3DSystematicPoking(
  std::span<const double> stretchKnots,
  int stretchRestKnotIndex,
  std::span<const double> volumeKnots,
  int volumeRestKnotIndex):
  ElasticModel3DValanisLandel(
    makeSystematicPokingStretchModel(
      stretchKnots, stretchRestKnotIndex),
    std::make_shared<ElasticModel1DZero>(),
    makeSystematicPokingVolumeModel(
      volumeKnots, volumeRestKnotIndex))
{
}

SystematicPokingDefinition::SystematicPokingDefinition(
  std::span<const double> stretchKnots,
  int stretchRestKnotIndex,
  std::span<const double> volumeKnots,
  int volumeRestKnotIndex):
  stretchKnots_(stretchKnots.begin(), stretchKnots.end()),
  stretchRestKnotIndex_(stretchRestKnotIndex),
  volumeKnots_(volumeKnots.begin(), volumeKnots.end()),
  volumeRestKnotIndex_(volumeRestKnotIndex),
  optimizableChannelSchema_(
    makeSystematicPokingOptimizableChannelSchema(
      stretchKnots_.size()))
{
  // Construct once so the definition and the evaluator share exactly the
  // same knot, anchor, and positive-volume validation contract.
  [[maybe_unused]] ElasticModel3DSystematicPoking validationModel(
    stretchKnots_,
    stretchRestKnotIndex_,
    volumeKnots_,
    volumeRestKnotIndex_);
}

MaterialChannelSchema
SystematicPokingDefinition::fixedChannelSchema() const
{
  return {};
}

MaterialChannelSchema
SystematicPokingDefinition::optimizableChannelSchema() const
{
  return optimizableChannelSchema_;
}

std::unique_ptr<ElasticModel>
SystematicPokingDefinition::createModel(
  std::span<const double> fixedChannels,
  const MaterialFrame &) const
{
  if (!fixedChannels.empty())
    throw std::invalid_argument(
      "systematic_poking has no fixed material channels");
  return std::make_unique<ElasticModel3DSystematicPoking>(
    stretchKnots_,
    stretchRestKnotIndex_,
    volumeKnots_,
    volumeRestKnotIndex_);
}

}  // namespace pgo::SolidDeformationModel
