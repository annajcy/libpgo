#include <gtest/gtest.h>

#include "factories/elasticModelFactory.h"

#include "simulationMesh.h"
#include "elasticModelStableNeoHookeanMaterial.h"
#include "elasticModelLinearMaterial.h"
#include "elasticModelInvariantBasedMaterial.h"
#include "elasticModel3DSTVKMaterial.h"
#include "elasticModel3DMooneyRivlin.h"
#include "elasticModelCombinedMaterial.h"
#include "cubicMesh.h"
#include "tetMesh.h"
#include "pgoLogging.h"

namespace
{
using namespace pgo::SolidDeformationModel;

constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;
}  // namespace

TEST(ElasticModelFactoryGTest, CreateStableNeo)
{
  pgo::Logging::init();
  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);

  auto result = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::STABLE_NEO, nullptr);

  ASSERT_NE(result, nullptr);
  EXPECT_NE(dynamic_cast<ElasticModelStableNeoHookeanMaterial *>(result.get()), nullptr);
}

TEST(ElasticModelFactoryGTest, CreateLinear)
{
  pgo::Logging::init();
  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);

  auto result = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::LINEAR, nullptr);

  ASSERT_NE(result, nullptr);
  EXPECT_NE(dynamic_cast<ElasticModelLinearMaterial *>(result.get()), nullptr);
}

TEST(ElasticModelFactoryGTest, CreateStVK)
{
  pgo::Logging::init();
  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);

  auto result = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::STVK, nullptr);

  ASSERT_NE(result, nullptr);
  EXPECT_NE(dynamic_cast<ElasticModel3DSTVKMaterial *>(result.get()), nullptr);
}

TEST(ElasticModelFactoryGTest, UnknownElasticModelThrows)
{
  pgo::Logging::init();
  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);

  auto invalidType = static_cast<DeformationModelElasticMaterial>(999);
  EXPECT_THROW(
    ElasticModelFactory::create(*simMesh, 0, invalidType, nullptr),
    std::runtime_error);
}
