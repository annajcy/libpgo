#include <gtest/gtest.h>

#include "deformationModelState.h"
#include "formulations/parameters/parameterField.h"
#include "simulationMesh.h"
#include "cubicMesh.h"
#include "pgoLogging.h"

#include <memory>

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;

std::shared_ptr<const SimulationMesh> makeCubicSimulationMesh()
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  return std::shared_ptr<const SimulationMesh>(loadCubicMesh(&cubicMesh).release());
}
}  // namespace

TEST(DeformationModelStateGTest, CreatesDefaultElementwiseFieldsFromOneMesh)
{
  pgo::Logging::init();

  auto mesh = makeCubicSimulationMesh();

  auto state = DeformationModelState::create(
    mesh,
    DeformationModelElasticMaterial::STABLE_NEO,
    ElasticFieldInit{},
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    PlasticFieldInit{});

  ASSERT_NE(state, nullptr);
  EXPECT_EQ(state->mesh().get(), mesh.get());
  EXPECT_EQ(state->elasticMaterial(), DeformationModelElasticMaterial::STABLE_NEO);
  EXPECT_EQ(state->plasticMaterial(), DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);
  EXPECT_EQ(state->elasticField().spec().domain, ParameterDomain::ELASTIC);
  EXPECT_EQ(state->plasticField().spec().domain, ParameterDomain::PLASTIC);
  EXPECT_EQ(state->elasticField().dofLayout()->numGlobalDofs(), mesh->getNumElements() * 2);
  EXPECT_EQ(state->plasticField().dofLayout()->numGlobalDofs(), mesh->getNumElements() * 6);
}

TEST(DeformationModelStateGTest, OwnsAndUpdatesElementwisePlasticValues)
{
  pgo::Logging::init();

  auto mesh = makeCubicSimulationMesh();
  ES::VXd plasticValues = ES::VXd::Zero(mesh->getNumElements() * 6);
  for (int ei = 0; ei < mesh->getNumElements(); ei++) {
    plasticValues.segment<6>(ei * 6) << 1.05, 0.0, 0.0, 1.0, 0.0, 1.0;
  }

  auto state = DeformationModelState::create(
    mesh,
    DeformationModelElasticMaterial::STABLE_NEO,
    ElasticFieldInit{},
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, plasticValues });

  EXPECT_TRUE(state->plasticParameterSnapshot().isApprox(plasticValues));

  ES::VXd updated = plasticValues;
  updated[0] = 0.95;
  state->setPlasticValues(updated);
  EXPECT_TRUE(state->plasticParameterSnapshot().isApprox(updated));
}

TEST(DeformationModelStateGTest, CreatesConstantFieldsSharedAcrossMesh)
{
  pgo::Logging::init();

  auto mesh = makeCubicSimulationMesh();

  auto state = DeformationModelState::create(
    mesh,
    DeformationModelElasticMaterial::STABLE_NEO,
    ElasticFieldInit{ ElasticMaterialFieldType::CONSTANT, std::nullopt },
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    PlasticFieldInit{ PlasticMaterialFieldType::CONSTANT, std::nullopt });

  ASSERT_NE(state, nullptr);
  // A constant (mesh-wide shared) field has numChannels global dofs, not
  // numChannels * numElements.
  EXPECT_EQ(state->elasticField().dofLayout()->numGlobalDofs(), 2);
  EXPECT_EQ(state->plasticField().dofLayout()->numGlobalDofs(), 6);
  EXPECT_EQ(state->elasticField().kind(), ParameterFieldKind::CONSTANT);
  EXPECT_EQ(state->plasticField().kind(), ParameterFieldKind::CONSTANT);
}

TEST(DeformationModelStateGTest, OwnsAndUpdatesConstantPlasticValues)
{
  pgo::Logging::init();

  auto mesh = makeCubicSimulationMesh();
  ES::VXd plasticValues(6);
  plasticValues << 1.05, 0.0, 0.0, 1.0, 0.0, 1.0;

  auto state = DeformationModelState::create(
    mesh,
    DeformationModelElasticMaterial::STABLE_NEO,
    ElasticFieldInit{},
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    PlasticFieldInit{ PlasticMaterialFieldType::CONSTANT, plasticValues });

  EXPECT_EQ(state->plasticParameterSnapshot().size(), 6);
  EXPECT_TRUE(state->plasticParameterSnapshot().isApprox(plasticValues));

  ES::VXd updated = plasticValues;
  updated[0] = 0.95;
  state->setPlasticValues(updated);
  EXPECT_TRUE(state->plasticParameterSnapshot().isApprox(updated));
}

TEST(DeformationModelStateGTest, RejectsWrongConstantValueSize)
{
  pgo::Logging::init();

  auto mesh = makeCubicSimulationMesh();
  ES::VXd wrongPlastic = ES::VXd::Zero(mesh->getNumElements() * 6);

  EXPECT_THROW(
    DeformationModelState::create(
      mesh,
      DeformationModelElasticMaterial::STABLE_NEO,
      ElasticFieldInit{},
      DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
      PlasticFieldInit{ PlasticMaterialFieldType::CONSTANT, wrongPlastic }),
    std::invalid_argument);
}

TEST(DeformationModelStateGTest, RejectsWrongElementwiseValueSize)
{
  pgo::Logging::init();

  auto mesh = makeCubicSimulationMesh();
  ES::VXd wrongPlastic = ES::VXd::Zero(5);

  EXPECT_THROW(
    DeformationModelState::create(
      mesh,
      DeformationModelElasticMaterial::STABLE_NEO,
      ElasticFieldInit{},
      DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
      PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, wrongPlastic }),
    std::invalid_argument);
}
