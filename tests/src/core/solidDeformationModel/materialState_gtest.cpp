#include <gtest/gtest.h>
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "material/plastic/plasticModel3D3DOF.h"
#include "material/plastic/plasticModel3D6DOF.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"

#include "material/runtime/materialState.h"
#include "materialTestUtils.h"
#include "cubicMesh.h"

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;

TEST(MaterialState, StoresImmutableElasticAndPlasticValues)
{
  ES::VXd plasticValues(6);
  plasticValues << 1.0, 0.01, 0.02, 0.99, 0.03, 1.01;
  auto parameters = std::make_shared<MaterialState>(
    ES::VXd::Zero(0), plasticValues);

  EXPECT_EQ(parameters->elasticValues().size(), 0);
  EXPECT_EQ(parameters->plasticValues().size(), 6);
  EXPECT_TRUE(parameters->plasticValues().isApprox(plasticValues));

  ES::VXd changed = plasticValues * 1.1;
  const MaterialState changedState = parameters->withPlasticValues(
    std::span<const double>(changed.data(), changed.size()));
  EXPECT_TRUE(parameters->plasticValues().isApprox(plasticValues));
  EXPECT_TRUE(changedState.plasticValues().isApprox(changed));
}

TEST(MaterialState, LengthValidationBelongsToTheConsumer)
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  auto asset = TestUtils::shareAsset(loadCubicMesh(cubicMesh));
  const auto &mesh = asset->mesh();

  auto elementwise = TestUtils::makeDefaultMaterialState(
    *asset,
    *std::make_shared<StableNeoDefinition>(),
    *std::make_shared<VolumetricPlasticity6Definition>());
  EXPECT_EQ(
    elementwise->plasticValues().size(),
    mesh->getNumElements() * 6);

  EXPECT_NO_THROW(MaterialState(ES::VXd::Zero(0), ES::VXd::Zero(5)));
}

}  // namespace
