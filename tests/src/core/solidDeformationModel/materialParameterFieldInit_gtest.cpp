#include <gtest/gtest.h>

#include "material/fields/materialParameterFieldInit.h"
#include "material/fields/parameterField.h"
#include "simulation/simulationMesh.h"
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

ES::VXd snapshot(const OptimizableField &field)
{
  const int n = field.dofLayout()->numGlobalDofs();
  ES::VXd values(n);
  if (n > 0)
    values = Eigen::Map<const ES::VXd>(field.globalData(), n);
  return values;
}
}  // namespace

TEST(MaterialParameterFieldInitGTest, CreatesDefaultElementwiseFieldsFromOneMesh)
{
  pgo::Logging::init();

  auto mesh = makeCubicSimulationMesh();

  auto elasticField = createElasticParameterField(*mesh, DeformationModelElasticMaterial::STABLE_NEO, ElasticFieldInit{});
  auto plasticField = createPlasticParameterField(*mesh, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, PlasticFieldInit{});

  ASSERT_NE(elasticField, nullptr);
  ASSERT_NE(plasticField, nullptr);
  EXPECT_EQ(elasticField->spec().domain, ParameterDomain::ELASTIC);
  EXPECT_EQ(plasticField->spec().domain, ParameterDomain::PLASTIC);
  // STABLE_NEO exposes no differentiable elastic parameters, so its elastic field
  // has 0 channels (matches getNumParameters()); the plastic DOF6 field has 6.
  EXPECT_EQ(elasticField->dofLayout()->numGlobalDofs(), 0);
  EXPECT_EQ(plasticField->dofLayout()->numGlobalDofs(), mesh->getNumElements() * 6);
}

TEST(MaterialParameterFieldInitGTest, OwnsAndUpdatesElementwisePlasticValues)
{
  pgo::Logging::init();

  auto mesh = makeCubicSimulationMesh();
  ES::VXd plasticValues = ES::VXd::Zero(mesh->getNumElements() * 6);
  for (int ei = 0; ei < mesh->getNumElements(); ei++) {
    plasticValues.segment<6>(ei * 6) << 1.05, 0.0, 0.0, 1.0, 0.0, 1.0;
  }

  auto plasticField = createPlasticParameterField(
    *mesh,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, plasticValues });

  EXPECT_TRUE(snapshot(*plasticField).isApprox(plasticValues));

  ES::VXd updated = plasticValues;
  updated[0] = 0.95;
  plasticField->setGlobalData(updated.data());
  EXPECT_TRUE(snapshot(*plasticField).isApprox(updated));
}

TEST(MaterialParameterFieldInitGTest, CreatesConstantFieldsSharedAcrossMesh)
{
  pgo::Logging::init();

  auto mesh = makeCubicSimulationMesh();

  auto elasticField = createElasticParameterField(
    *mesh,
    DeformationModelElasticMaterial::STABLE_NEO,
    ElasticFieldInit{ ElasticMaterialFieldType::CONSTANT, std::nullopt });
  auto plasticField = createPlasticParameterField(
    *mesh,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    PlasticFieldInit{ PlasticMaterialFieldType::CONSTANT, std::nullopt });

  ASSERT_NE(elasticField, nullptr);
  ASSERT_NE(plasticField, nullptr);
  // A constant (mesh-wide shared) field has numChannels global dofs, not
  // numChannels * numElements. STABLE_NEO has 0 elastic channels; the constant
  // sharing semantics are exercised by the plastic DOF6 field (6 shared dofs).
  EXPECT_EQ(elasticField->dofLayout()->numGlobalDofs(), 0);
  EXPECT_EQ(plasticField->dofLayout()->numGlobalDofs(), 6);
  EXPECT_EQ(elasticField->kind(), ParameterFieldKind::CONSTANT);
  EXPECT_EQ(plasticField->kind(), ParameterFieldKind::CONSTANT);
}

TEST(MaterialParameterFieldInitGTest, OwnsAndUpdatesConstantPlasticValues)
{
  pgo::Logging::init();

  auto mesh = makeCubicSimulationMesh();
  ES::VXd plasticValues(6);
  plasticValues << 1.05, 0.0, 0.0, 1.0, 0.0, 1.0;

  auto plasticField = createPlasticParameterField(
    *mesh,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    PlasticFieldInit{ PlasticMaterialFieldType::CONSTANT, plasticValues });

  EXPECT_EQ(snapshot(*plasticField).size(), 6);
  EXPECT_TRUE(snapshot(*plasticField).isApprox(plasticValues));

  ES::VXd updated = plasticValues;
  updated[0] = 0.95;
  plasticField->setGlobalData(updated.data());
  EXPECT_TRUE(snapshot(*plasticField).isApprox(updated));
}

TEST(MaterialParameterFieldInitGTest, RejectsWrongConstantValueSize)
{
  pgo::Logging::init();

  auto mesh = makeCubicSimulationMesh();
  ES::VXd wrongPlastic = ES::VXd::Zero(mesh->getNumElements() * 6);

  EXPECT_THROW(
    createPlasticParameterField(
      *mesh,
      DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
      PlasticFieldInit{ PlasticMaterialFieldType::CONSTANT, wrongPlastic }),
    std::invalid_argument);
}

TEST(MaterialParameterFieldInitGTest, RejectsWrongElementwiseValueSize)
{
  pgo::Logging::init();

  auto mesh = makeCubicSimulationMesh();
  ES::VXd wrongPlastic = ES::VXd::Zero(5);

  EXPECT_THROW(
    createPlasticParameterField(
      *mesh,
      DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
      PlasticFieldInit{ PlasticMaterialFieldType::ELEMENTWISE, wrongPlastic }),
    std::invalid_argument);
}
