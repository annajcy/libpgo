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

#include <stdexcept>
#include <string>

namespace
{
using namespace pgo::SolidDeformationModel;

constexpr const char *kTorusVegPath = LIBPGO_TEST_TORUS_VEG;

std::unique_ptr<SimulationMesh> makeSingleTetSimulationMesh()
{
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3 };
  const int elementMaterialIndices[] = { 0 };
  SimulationMeshENuMaterial baseMaterial(1200.0, 0.45);
  const SimulationMeshMaterial *materials[] = { &baseMaterial };

  return std::make_unique<SimulationMesh>(
    4, vertices,
    1, 4, elementVertices,
    elementMaterialIndices, 1, materials,
    SimulationMeshType::TET);
}

std::unique_ptr<SimulationMesh> makeSingleShellSimulationMesh()
{
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    -0.2, 0.0, 0.0,
    0.8, 0.2, 0.0,
    0.2, 0.8, 0.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3, 4, 5 };
  const int elementMaterialIndices[] = { 0 };
  SimulationMeshENuhMaterial shellMaterial(1200.0, 0.45, 1e-3);
  const SimulationMeshMaterial *materials[] = { &shellMaterial };

  return std::make_unique<SimulationMesh>(
    6, vertices,
    1, 6, elementVertices,
    elementMaterialIndices, 1, materials,
    SimulationMeshType::SHELL);
}
}  // namespace

TEST(ElasticModelFactoryGTest, CreateStableNeo)
{
  pgo::Logging::init();
  pgo::VolumetricMeshes::TetMesh tetMesh(kTorusVegPath);
  auto simMesh = loadTetMesh(&tetMesh);
  ASSERT_NE(simMesh, nullptr);
  ASSERT_EQ(simMesh->getElementNumMaterials(0), 1);

  testing::internal::CaptureStdout();
  auto result = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::STABLE_NEO, nullptr);
  const std::string stdoutText = testing::internal::GetCapturedStdout();

  ASSERT_NE(result, nullptr);
  EXPECT_NE(dynamic_cast<ElasticModelStableNeoHookeanMaterial *>(result.get()), nullptr);
  EXPECT_EQ(stdoutText.find("j >= 0 && j < getElementNumMaterials"), std::string::npos);
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
    std::logic_error);
}

TEST(ElasticModelFactoryGTest, HillParameterSpecMatchesCreatedModelParameters)
{
  pgo::Logging::init();
  auto simMesh = makeSingleTetSimulationMesh();
  SimulationMeshHillMaterial hillMaterial(1e5, 1.2, 0.7);
  simMesh->appendMaterialToAllElements(&hillMaterial);

  const double fiberDirection[3] = { 1.0, 0.0, 0.0 };
  auto result = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::HILL_STABLE_NEO, fiberDirection);
  ASSERT_NE(result, nullptr);

  const auto spec = ElasticModelFactory::parameterSpec(
    *simMesh, DeformationModelElasticMaterial::HILL_STABLE_NEO);
  EXPECT_EQ(simMesh->getElementMaterial(0, 0)->numElasticParameters(
    DeformationModelElasticMaterial::HILL_STABLE_NEO), result->getNumParameters());
  EXPECT_EQ(spec.numChannels, result->getNumParameters());
  ASSERT_EQ(spec.channelNames.size(), 1);
  EXPECT_EQ(spec.channelNames[0], "activation");
}

TEST(ElasticModelFactoryGTest, KoiterFabricParameterSpecMatchesCreatedModelParameters)
{
  pgo::Logging::init();
  auto simMesh = makeSingleShellSimulationMesh();

  auto result = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::KOITER_FABRIC, nullptr);
  ASSERT_NE(result, nullptr);

  const auto spec = ElasticModelFactory::parameterSpec(
    *simMesh, DeformationModelElasticMaterial::KOITER_FABRIC);
  EXPECT_EQ(simMesh->getElementMaterial(0, 0)->numElasticParameters(
    DeformationModelElasticMaterial::KOITER_FABRIC), result->getNumParameters());
  EXPECT_EQ(spec.numChannels, result->getNumParameters());
  EXPECT_EQ(spec.channelNames.size(), static_cast<size_t>(result->getNumParameters()));
}

TEST(ElasticModelFactoryGTest, HillDefaultFieldsUseSingleActivationChannel)
{
  pgo::Logging::init();
  auto simMesh = makeSingleTetSimulationMesh();
  SimulationMeshHillMaterial hillMaterial(1e5, 1.2, 0.7);
  simMesh->appendMaterialToAllElements(&hillMaterial);

  auto elementwiseField = ElasticModelFactory::createDefaultElementwiseField(
    *simMesh, DeformationModelElasticMaterial::HILL_STABLE_NEO);
  ASSERT_NE(elementwiseField, nullptr);
  ASSERT_NE(elementwiseField->dofLayout(), nullptr);
  EXPECT_EQ(elementwiseField->numChannels(), 1);
  EXPECT_EQ(elementwiseField->dofLayout()->numGlobalDofs(), simMesh->getNumElements());
  ASSERT_NE(elementwiseField->globalData(), nullptr);
  EXPECT_DOUBLE_EQ(elementwiseField->globalData()[0], 1.0);

  auto constantField = ElasticModelFactory::createDefaultConstantField(
    *simMesh, DeformationModelElasticMaterial::HILL_STABLE_NEO);
  ASSERT_NE(constantField, nullptr);
  ASSERT_NE(constantField->dofLayout(), nullptr);
  EXPECT_EQ(constantField->numChannels(), 1);
  EXPECT_EQ(constantField->dofLayout()->numGlobalDofs(), 1);
  ASSERT_NE(constantField->globalData(), nullptr);
  EXPECT_DOUBLE_EQ(constantField->globalData()[0], 1.0);
}

TEST(ElasticModelFactoryGTest, KoiterFabricDefaultFieldsUseModelParameterChannels)
{
  pgo::Logging::init();
  auto simMesh = makeSingleShellSimulationMesh();

  auto elementwiseField = ElasticModelFactory::createDefaultElementwiseField(
    *simMesh, DeformationModelElasticMaterial::KOITER_FABRIC);
  ASSERT_NE(elementwiseField, nullptr);
  ASSERT_NE(elementwiseField->dofLayout(), nullptr);
  EXPECT_EQ(elementwiseField->numChannels(), 12);
  EXPECT_EQ(elementwiseField->dofLayout()->numGlobalDofs(), 12 * simMesh->getNumElements());
  ASSERT_NE(elementwiseField->globalData(), nullptr);
  EXPECT_DOUBLE_EQ(elementwiseField->globalData()[0], 1.0);
  EXPECT_DOUBLE_EQ(elementwiseField->globalData()[11], 1e-3);

  auto constantField = ElasticModelFactory::createDefaultConstantField(
    *simMesh, DeformationModelElasticMaterial::KOITER_FABRIC);
  ASSERT_NE(constantField, nullptr);
  ASSERT_NE(constantField->dofLayout(), nullptr);
  EXPECT_EQ(constantField->numChannels(), 12);
  EXPECT_EQ(constantField->dofLayout()->numGlobalDofs(), 12);
  ASSERT_NE(constantField->globalData(), nullptr);
  EXPECT_DOUBLE_EQ(constantField->globalData()[0], 1.0);
  EXPECT_DOUBLE_EQ(constantField->globalData()[11], 1e-3);
}
