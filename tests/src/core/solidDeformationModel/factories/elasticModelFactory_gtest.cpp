#include <gtest/gtest.h>

#include "material/elastic/elasticModelFactory.h"
#include "material/fields/materialParameterFactory.h"

#include "simulation/simulationMesh.h"
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModelLinearMaterial.h"
#include "material/elastic/elasticModelInvariantBasedMaterial.h"
#include "material/elastic/elasticModel3DSTVKMaterial.h"
#include "material/elastic/elasticModel3DMooneyRivlin.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModelHillTypeMaterial.h"
#include "cubicMesh.h"
#include "tetMesh.h"
#include "pgoLogging.h"

#include <memory>
#include <optional>
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
    *simMesh, 0, DeformationModelElasticMaterial::STABLE_NEO,
    MaterialFrame::Identity());
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
    *simMesh, 0, DeformationModelElasticMaterial::LINEAR,
    MaterialFrame::Identity());

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
    *simMesh, 0, DeformationModelElasticMaterial::STVK,
    MaterialFrame::Identity());

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
    ElasticModelFactory::create(
      *simMesh, 0, invalidType, MaterialFrame::Identity()),
    std::logic_error);
}

TEST(ElasticModelFactoryGTest, HillParameterSpecMatchesCreatedModelParameters)
{
  pgo::Logging::init();
  auto simMesh = makeSingleTetSimulationMesh();
  SimulationMeshHillMaterial hillMaterial(1e5, 1.2, 0.7);
  simMesh->appendMaterialToAllElements(&hillMaterial);

  constexpr double kInvSqrt2 = 0.7071067811865475244;
  MaterialFrame frame;
  frame.col(0) << kInvSqrt2, kInvSqrt2, 0.0;
  frame.col(1) << -kInvSqrt2, kInvSqrt2, 0.0;
  frame.col(2) << 0.0, 0.0, 1.0;
  auto result = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::HILL_STABLE_NEO, frame);
  ASSERT_NE(result, nullptr);
  auto *combined =
    dynamic_cast<ElasticModelCombinedMaterial<2> *>(result.get());
  ASSERT_NE(combined, nullptr);
  const auto *hill = dynamic_cast<const ElasticModelHillTypeMaterial *>(
    combined->getMaterial(1));
  ASSERT_NE(hill, nullptr);
  EXPECT_TRUE(hill->primaryAxis().isApprox(frame.col(0), 1e-12));

  const auto spec = ElasticModelFactory::parameterSpec(
    *simMesh, DeformationModelElasticMaterial::HILL_STABLE_NEO);
  EXPECT_EQ(spec.channelNames.size(), result->getNumParameters());
  ASSERT_EQ(spec.channelNames.size(), 1);
  EXPECT_EQ(spec.channelNames[0], "activation");
}

TEST(ElasticModelFactoryGTest, KoiterFabricParameterSpecMatchesCreatedModelParameters)
{
  pgo::Logging::init();
  auto simMesh = makeSingleShellSimulationMesh();

  auto result = ElasticModelFactory::create(
    *simMesh, 0, DeformationModelElasticMaterial::KOITER_FABRIC,
    MaterialFrame::Identity());
  ASSERT_NE(result, nullptr);

  const auto spec = ElasticModelFactory::parameterSpec(
    *simMesh, DeformationModelElasticMaterial::KOITER_FABRIC);
  EXPECT_EQ(spec.channelNames.size(), result->getNumParameters());
  EXPECT_EQ(spec.channelNames.size(), static_cast<size_t>(result->getNumParameters()));
}

TEST(ElasticModelFactoryGTest, HillDefaultParametersUseSingleActivationChannel)
{
  pgo::Logging::init();
  auto simMesh = makeSingleTetSimulationMesh();
  SimulationMeshHillMaterial hillMaterial(1e5, 1.2, 0.7);
  simMesh->appendMaterialToAllElements(&hillMaterial);

  auto elementwise = makeDefaultMaterialParameters(
    *simMesh, DeformationModelElasticMaterial::HILL_STABLE_NEO,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF0);
  EXPECT_EQ(elementwise->space()->elastic().mapping().numChannels(), 1);
  EXPECT_EQ(
    elementwise->space()->elastic().dofLayout().numGlobalDofs(),
    simMesh->getNumElements());
  EXPECT_DOUBLE_EQ(elementwise->elasticSnapshot()[0], 1.0);

  auto constant = makeMaterialParameters(
    *simMesh, DeformationModelElasticMaterial::HILL_STABLE_NEO,
    std::make_unique<ConstantParameterDofLayout>(
      simMesh->getNumElements(), 1),
    std::make_unique<IdentityParameterFieldMapping>(1), std::nullopt,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF0,
    std::make_unique<ElementwiseParameterDofLayout>(
      simMesh->getNumElements(), 0),
    std::make_unique<IdentityParameterFieldMapping>(0), std::nullopt);
  EXPECT_EQ(constant->space()->elastic().dofLayout().numGlobalDofs(), 1);
  EXPECT_DOUBLE_EQ(constant->elasticSnapshot()[0], 1.0);
}

TEST(ElasticModelFactoryGTest, KoiterFabricDefaultParametersUseModelChannels)
{
  pgo::Logging::init();
  auto simMesh = makeSingleShellSimulationMesh();

  auto elementwise = makeDefaultMaterialParameters(
    *simMesh, DeformationModelElasticMaterial::KOITER_FABRIC,
    DeformationModelPlasticMaterial::SHELL_FF_DOF0);
  EXPECT_EQ(elementwise->space()->elastic().mapping().numChannels(), 12);
  EXPECT_EQ(
    elementwise->space()->elastic().dofLayout().numGlobalDofs(),
    12 * simMesh->getNumElements());
  EXPECT_DOUBLE_EQ(elementwise->elasticSnapshot()[0], 1.0);
  EXPECT_DOUBLE_EQ(elementwise->elasticSnapshot()[11], 1e-3);

  auto constant = makeMaterialParameters(
    *simMesh, DeformationModelElasticMaterial::KOITER_FABRIC,
    std::make_unique<ConstantParameterDofLayout>(
      simMesh->getNumElements(), 12),
    std::make_unique<IdentityParameterFieldMapping>(12), std::nullopt,
    DeformationModelPlasticMaterial::SHELL_FF_DOF0,
    std::make_unique<ElementwiseParameterDofLayout>(
      simMesh->getNumElements(), 0),
    std::make_unique<IdentityParameterFieldMapping>(0), std::nullopt);
  EXPECT_EQ(constant->space()->elastic().dofLayout().numGlobalDofs(), 12);
  EXPECT_DOUBLE_EQ(constant->elasticSnapshot()[0], 1.0);
  EXPECT_DOUBLE_EQ(constant->elasticSnapshot()[11], 1e-3);
}
