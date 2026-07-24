#include <gtest/gtest.h>

#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModelHillTypeMaterial.h"
#include "material/plastic/plasticModel3D3DOF.h"

using namespace pgo::SolidDeformationModel;

TEST(ElasticModelConfig, StableNeoHasStableIdentity)
{
  StableNeoConfig config;
  EXPECT_EQ(config.id(), "stable_neo");
  EXPECT_TRUE(config.parameterSpec().channelNames.empty());
  EXPECT_EQ(config.frameRequirement(), MaterialFrameRequirement::None);
}

TEST(ElasticModelConfig, HillRequiresActivationAndPrimaryAxis)
{
  HillStableNeoConfig config;
  EXPECT_EQ(config.parameterSpec().channelNames.size(), 1);
  EXPECT_EQ(config.parameterSpec().channelNames.front(), "activation");
  EXPECT_EQ(config.frameRequirement(), MaterialFrameRequirement::PrimaryAxis);
}

TEST(PlasticModelConfig, DofVariantsExposeExplicitConfigs)
{
  VolumetricPlasticity3Config config;
  EXPECT_EQ(config.id(), "volumetric_dof3");
  EXPECT_EQ(config.parameterSpec().channelNames.size(), 3);
  EXPECT_EQ(config.frameRequirement(), MaterialFrameRequirement::FullFrame);
}
