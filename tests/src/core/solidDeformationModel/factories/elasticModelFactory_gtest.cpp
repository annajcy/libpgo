#include <gtest/gtest.h>

#include "factories/elasticModelFactory.h"

#include "simulationMesh.h"
#include "elasticModelStableNeoHookeanMaterial.h"
#include "elasticModelLinearMaterial.h"
#include "elasticModelHillTypeMaterial.h"
#include "elasticModelInvariantBasedMaterial.h"
#include "elasticModelVolumeMaterial.h"
#include "elasticModel3DSTVKMaterial.h"
#include "elasticModel3DMooneyRivlin.h"
#include "elasticModelCombinedMaterial.h"
#include "elasticModel2DFundamentalFormsFabric.h"
#include "elasticModel2DFundamentalFormsSTVK.h"
#include "invariantBasedMaterialStVK.h"
#include "cubicMesh.h"
#include "tetMesh.h"
#include "pgoLogging.h"

namespace
{
using namespace pgo::SolidDeformationModel;

constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;
constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;
}  // namespace

// Test that the factory creates StableNeo from ENu payload.
TEST(ElasticModelFactoryGTest, CreateStableNeo)
{
  pgo::Logging::init();
  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);

  auto result = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::STABLE_NEO, nullptr);

  ASSERT_NE(result.elementMaterial, nullptr);
  ASSERT_NE(result.stableNeo, nullptr);
  EXPECT_EQ(result.elementMaterial, result.stableNeo);

  // All other typed pointers must be null.
  EXPECT_EQ(result.linear, nullptr);
  EXPECT_EQ(result.hill, nullptr);
  EXPECT_EQ(result.invariantBased, nullptr);
  EXPECT_EQ(result.volume, nullptr);
  EXPECT_EQ(result.stvk, nullptr);
  EXPECT_EQ(result.mooneyRivlin, nullptr);
  EXPECT_EQ(result.combined2, nullptr);
  EXPECT_EQ(result.combined3, nullptr);

  delete result.stableNeo;
}

// Test that the factory creates Linear from ENu payload.
TEST(ElasticModelFactoryGTest, CreateLinear)
{
  pgo::Logging::init();
  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);

  auto result = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::LINEAR, nullptr);

  ASSERT_NE(result.elementMaterial, nullptr);
  ASSERT_NE(result.linear, nullptr);
  EXPECT_EQ(result.elementMaterial, result.linear);

  delete result.linear;
}

// Test that the factory creates StVK from ENu payload.
TEST(ElasticModelFactoryGTest, CreateStVK)
{
  pgo::Logging::init();
  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);

  auto result = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::STVK, nullptr);

  ASSERT_NE(result.elementMaterial, nullptr);
  ASSERT_NE(result.stvk, nullptr);
  EXPECT_EQ(result.elementMaterial, result.stvk);

  delete result.stvk;
}

// Test that unknown elastic model enum throws.
TEST(ElasticModelFactoryGTest, UnknownElasticModelThrows)
{
  pgo::Logging::init();
  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);

  // Cast an invalid enum value.
  auto invalidType = static_cast<DeformationModelElasticMaterial>(999);
  EXPECT_THROW(
    ElasticModelFactory::create(*simMesh, 0, invalidType, nullptr),
    std::runtime_error);
}
