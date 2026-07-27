#include <gtest/gtest.h>
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "material/plastic/plasticModel3D3DOF.h"
#include "material/plastic/plasticModel3D6DOF.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"

#include "material/runtime/optimizableParameters.h"
#include "materialTestUtils.h"
#include "cubicMesh.h"

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;

TEST(OptimizableParameters, BuildsIndependentFieldsAndCommittedValues)
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto asset = TestUtils::shareAsset(loadCubicMesh(cubicMesh));
  ASSERT_NE(asset, nullptr);
  const auto &mesh = asset->mesh();

  const auto elastic = std::make_shared<StableNeoDefinition>();
  const auto plastic = std::make_shared<VolumetricPlasticity6Definition>();
  ES::VXd plasticValues(6);
  plasticValues << 1.0, 0.01, 0.02, 0.99, 0.03, 1.01;

  auto elasticField = std::make_shared<const OptimizableParameterField>(
    TestUtils::identityParameterSchema(elastic->optimizableChannelSchema()),
    std::make_shared<ElementwiseParameterLayout>(
      mesh->getNumElements(), 0),
    std::make_shared<IdentityMaterialChannelMapping>(0));
  auto plasticField = std::make_shared<const OptimizableParameterField>(
    TestUtils::identityParameterSchema(plastic->optimizableChannelSchema()),
    std::make_shared<ConstantParameterLayout>(
      mesh->getNumElements(), 6),
    std::make_shared<IdentityMaterialChannelMapping>(6));
  auto parameters = std::make_shared<OptimizableParameters>(
    std::move(elasticField), std::move(plasticField),
    ES::VXd::Zero(0), plasticValues);

  EXPECT_EQ(
    parameters->elasticField().layout().numGlobalParameters(), 0);
  EXPECT_EQ(
    parameters->plasticField().layout().numGlobalParameters(), 6);
  EXPECT_TRUE(parameters->plasticSnapshot().isApprox(plasticValues));

  OptimizableParameterSnapshot snapshot = parameters->snapshot();
  ES::VXd changed = plasticValues * 1.1;
  parameters->setPlasticValues(changed);
  EXPECT_TRUE(Eigen::Map<const ES::VXd>(
    snapshot.view().plasticValues().data(),
    snapshot.view().plasticValues().size()).isApprox(plasticValues));
}

TEST(OptimizableParameters, ValidatesCommittedValueCountForAnyLayout)
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto asset = TestUtils::shareAsset(loadCubicMesh(cubicMesh));
  const auto &mesh = asset->mesh();

  auto elementwise = TestUtils::makeDefaultOptimizableParameters(
    *asset,
    *std::make_shared<StableNeoDefinition>(),
    *std::make_shared<VolumetricPlasticity6Definition>());
  EXPECT_EQ(
    elementwise->plasticSnapshot().size(),
    mesh->getNumElements() * 6);

  const StableNeoDefinition elastic;
  const VolumetricPlasticity6Definition plastic;
  auto elasticField = std::make_shared<const OptimizableParameterField>(
    TestUtils::identityParameterSchema(elastic.optimizableChannelSchema()),
    std::make_shared<ConstantParameterLayout>(mesh->getNumElements(), 0),
    std::make_shared<IdentityMaterialChannelMapping>(0));
  auto plasticField = std::make_shared<const OptimizableParameterField>(
    TestUtils::identityParameterSchema(plastic.optimizableChannelSchema()),
    std::make_shared<ConstantParameterLayout>(mesh->getNumElements(), 6),
    std::make_shared<IdentityMaterialChannelMapping>(6));
  EXPECT_THROW(
    OptimizableParameters(
      std::move(elasticField), std::move(plasticField),
      ES::VXd::Zero(0), ES::VXd::Zero(5)),
    std::invalid_argument);
}

TEST(OptimizableParameterField, RejectsDimensionMismatch)
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto asset = TestUtils::shareAsset(loadCubicMesh(cubicMesh));
  const auto &mesh = asset->mesh();

  EXPECT_THROW(
    OptimizableParameterField(
      ParameterInputSchema{},
      std::make_shared<ElementwiseParameterLayout>(mesh->getNumElements(), 1),
      std::make_shared<IdentityMaterialChannelMapping>(1)),
    std::invalid_argument);
}

}  // namespace
