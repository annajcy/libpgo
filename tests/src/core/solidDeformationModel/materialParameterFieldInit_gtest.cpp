#include <gtest/gtest.h>
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "material/plastic/plasticModel3D3DOF.h"
#include "material/plastic/plasticModel3D6DOF.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"

#include "material/core/materialParameterBuilder.h"
#include "simulation/simulationMesh.h"
#include "cubicMesh.h"

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;

TEST(MaterialParameterBuilder, BuildsIndependentSpaceAndCommittedValues)
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> mesh(loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(mesh, nullptr);

  const auto elastic = std::make_shared<StableNeoConfig>();
  const auto plastic = std::make_shared<VolumetricPlasticity6Config>();
  ES::VXd plasticValues(6);
  plasticValues << 1.0, 0.01, 0.02, 0.99, 0.03, 1.01;

  auto space = makeMaterialParameterSpace(
    *elastic,
    std::make_shared<ElementwiseParameterDofLayout>(
      mesh->getNumElements(), 0),
    std::make_shared<IdentityMaterialChannelMapping>(0),
    *plastic,
    std::make_shared<ConstantParameterDofLayout>(
      mesh->getNumElements(), 6),
    std::make_shared<IdentityMaterialChannelMapping>(6));
  auto parameters = makeMaterialParameters(
    std::move(space), ES::VXd::Zero(0), plasticValues);

  EXPECT_EQ(
    parameters->space()->elastic().dofLayout().numGlobalDofs(), 0);
  EXPECT_EQ(
    parameters->space()->plastic().dofLayout().numGlobalDofs(), 6);
  EXPECT_TRUE(parameters->plasticSnapshot().isApprox(plasticValues));

  MaterialParameterSnapshot snapshot = parameters->snapshot();
  ES::VXd changed = plasticValues * 1.1;
  parameters->setPlasticValues(changed);
  EXPECT_TRUE(Eigen::Map<const ES::VXd>(
    snapshot.view().plasticValues().data(),
    snapshot.view().plasticValues().size()).isApprox(plasticValues));
}

TEST(MaterialParameterBuilder, DefaultBuilderRequiresElementwiseLayout)
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> mesh(loadCubicMesh(&cubicMesh).release());

  auto elementwise = makeDefaultMaterialParameters(
    *mesh,
    *std::make_shared<StableNeoConfig>(),
    *std::make_shared<VolumetricPlasticity6Config>());
  EXPECT_EQ(
    elementwise->plasticSnapshot().size(),
    mesh->getNumElements() * 6);

  auto constantSpace = makeMaterialParameterSpace(
    *std::make_shared<StableNeoConfig>(),
    std::make_shared<ConstantParameterDofLayout>(mesh->getNumElements(), 0),
    std::make_shared<IdentityMaterialChannelMapping>(0),
    *std::make_shared<VolumetricPlasticity6Config>(),
    std::make_shared<ConstantParameterDofLayout>(mesh->getNumElements(), 6),
    std::make_shared<IdentityMaterialChannelMapping>(6));
  EXPECT_THROW(
    makeMaterialParameters(std::move(constantSpace), ES::VXd::Zero(0), ES::VXd::Zero(5)),
    std::invalid_argument);
}

TEST(MaterialParameterBuilder, RejectsDimensionMismatch)
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> mesh(loadCubicMesh(&cubicMesh).release());

  EXPECT_THROW(
    makeMaterialParameterSpace(
      *std::make_shared<StableNeoConfig>(),
      std::make_shared<ElementwiseParameterDofLayout>(mesh->getNumElements(), 1),
      std::make_shared<IdentityMaterialChannelMapping>(1),
      *std::make_shared<VolumetricPlasticity6Config>(),
      std::make_shared<ElementwiseParameterDofLayout>(mesh->getNumElements(), 6),
      std::make_shared<IdentityMaterialChannelMapping>(6)),
    std::invalid_argument);
}

}  // namespace
