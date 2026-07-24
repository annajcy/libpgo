#include <gtest/gtest.h>

#include "material/fields/materialParameterFactory.h"
#include "simulation/simulationMesh.h"
#include "cubicMesh.h"

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

constexpr const char *kCubicBoxVegPath = LIBPGO_TEST_CUBIC_BOX_VEG;

TEST(MaterialParameterFactory, BuildsIndependentSpaceAndCommittedValues)
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> mesh(loadCubicMesh(&cubicMesh).release());
  ASSERT_NE(mesh, nullptr);

  constexpr auto elastic = DeformationModelElasticMaterial::STABLE_NEO;
  constexpr auto plastic = DeformationModelPlasticMaterial::VOLUMETRIC_DOF6;
  ES::VXd plasticValues(6);
  plasticValues << 1.0, 0.01, 0.02, 0.99, 0.03, 1.01;

  auto parameters = makeMaterialParameters(
    *mesh,
    elastic,
    std::make_unique<ElementwiseParameterDofLayout>(
      mesh->getNumElements(), 0),
    std::make_unique<IdentityParameterFieldMapping>(0),
    std::nullopt,
    plastic,
    std::make_unique<ConstantParameterDofLayout>(
      mesh->getNumElements(), 6),
    std::make_unique<IdentityParameterFieldMapping>(6),
    plasticValues);

  EXPECT_EQ(
    parameters->space()->elastic().dofLayout().numGlobalDofs(), 0);
  EXPECT_EQ(
    parameters->space()->plastic().dofLayout().numGlobalDofs(), 6);
  EXPECT_TRUE(parameters->plasticSnapshot().isApprox(plasticValues));

  MaterialState snapshot = parameters->snapshot();
  ES::VXd changed = plasticValues * 1.1;
  parameters->setPlasticValues(changed);
  EXPECT_TRUE(Eigen::Map<const ES::VXd>(
    snapshot.view().plasticValues().data(),
    snapshot.view().plasticValues().size()).isApprox(plasticValues));
}

TEST(MaterialParameterFactory, DefaultsRespectLayoutOwnership)
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> mesh(loadCubicMesh(&cubicMesh).release());

  auto elementwise = makeDefaultMaterialParameters(
    *mesh,
    DeformationModelElasticMaterial::STABLE_NEO,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6);
  EXPECT_EQ(
    elementwise->plasticSnapshot().size(),
    mesh->getNumElements() * 6);

  auto constant = makeMaterialParameters(
    *mesh,
    DeformationModelElasticMaterial::STABLE_NEO,
    std::make_unique<ConstantParameterDofLayout>(
      mesh->getNumElements(), 0),
    std::make_unique<IdentityParameterFieldMapping>(0),
    std::nullopt,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    std::make_unique<ConstantParameterDofLayout>(
      mesh->getNumElements(), 6),
    std::make_unique<IdentityParameterFieldMapping>(6),
    std::nullopt);
  ASSERT_EQ(constant->plasticSnapshot().size(), 6);
  EXPECT_TRUE(constant->plasticSnapshot().isApprox(
    (ES::VXd(6) << 1, 0, 0, 1, 0, 1).finished()));
}

TEST(MaterialParameterFactory, RejectsDimensionMismatch)
{
  pgo::VolumetricMeshes::CubicMesh cubicMesh(kCubicBoxVegPath);
  std::shared_ptr<const SimulationMesh> mesh(loadCubicMesh(&cubicMesh).release());

  EXPECT_THROW(
    makeMaterialParameters(
      *mesh,
      DeformationModelElasticMaterial::STABLE_NEO,
      std::make_unique<ElementwiseParameterDofLayout>(
        mesh->getNumElements(), 1),
      std::make_unique<IdentityParameterFieldMapping>(1),
      ES::VXd::Zero(mesh->getNumElements()),
      DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
      std::make_unique<ElementwiseParameterDofLayout>(
        mesh->getNumElements(), 6),
      std::make_unique<IdentityParameterFieldMapping>(6),
      std::nullopt),
    std::invalid_argument);
}

}  // namespace
