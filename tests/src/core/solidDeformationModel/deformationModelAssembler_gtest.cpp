#include <gtest/gtest.h>
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "material/plastic/plasticModel3D3DOF.h"
#include "material/plastic/plasticModel3D6DOF.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"

#include "constraints/prescribedPrincipleStressConstraintFunctions.h"
#include "deformation/deformationModelAssembler.h"
#include "formulations/formulation/formulations.h"
#include "material/runtime/materialState.h"
#include "simulation/simulationMesh.h"
#include "triMeshGeo.h"
#include "materialTestUtils.h"

#include <cmath>
#include <algorithm>
#include <future>
#include <memory>
#include <span>
#include <stdexcept>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

template<typename Derived>
std::span<const double> constSpan(const Eigen::MatrixBase<Derived> &values)
{
  return std::span<const double>(values.derived().data(),
    static_cast<size_t>(values.size()));
}

template<typename Derived>
std::span<double> mutableSpan(Eigen::MatrixBase<Derived> &values)
{
  return std::span<double>(values.derived().data(),
    static_cast<size_t>(values.size()));
}

struct Fixture
{
  std::shared_ptr<const TestUtils::TestAsset> asset;
  std::shared_ptr<const SimulationMesh> mesh;
  std::shared_ptr<MaterialState> parameters;
  std::unique_ptr<DeformationModelAssembler> assembler;
  ES::VXd absolutePositions;
};

Fixture makeFixture()
{
  const double vertices[] = {
    0,
    0,
    0,
    1,
    0,
    0,
    1,
    1,
    0,
    0,
    1,
    0,
    0,
    0,
    1,
    1,
    0,
    1,
    1,
    1,
    1,
    0,
    1,
    1,
  };
  const int elementVertices[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  Fixture fixture;
  fixture.mesh = std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    8, vertices, 1, 8, elementVertices,
    SimulationMeshType::CUBIC));
  fixture.asset = TestUtils::makeENuAsset(
    fixture.mesh, 1200.0, 0.45);

  ES::VXd z(6);
  z << 1.01, 0.004, 0.003, 0.995, 0.005, 1.008;
  fixture.parameters = std::make_shared<MaterialState>(
    ES::VXd(), z);

  CubicLinearFormulation formulation;
  auto material = TestUtils::makeMaterialBinding(
    fixture.asset,
    std::make_shared<StableNeoDefinition>(),
    std::make_shared<VolumetricPlasticity6Definition>(),
    fixture.parameters);
  fixture.assembler = std::make_unique<DeformationModelAssembler>(
    fixture.asset->mesh(), material.binding, formulation, false);
  fixture.absolutePositions = fixture.assembler->getRestDofs();
  for (int i = 0; i < fixture.absolutePositions.size(); i++)
    fixture.absolutePositions[i] += 0.004 * std::sin(0.7 * i + 0.2);
  return fixture;
}

Fixture makeNonlinearShellFixture()
{
  const double vertices[] = {
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
  };
  const int triangles[] = { 0, 1, 2 };
  pgo::Mesh::TriMeshGeo surfaceMesh(3, vertices, 1, triangles);
  Fixture fixture;
  fixture.asset = TestUtils::makeENuhAsset(
    loadShellMesh(surfaceMesh), 1000.0, 0.45, 1e-3);
  fixture.mesh = fixture.asset->mesh();

  ES::VXd elastic(5);
  elastic << 2.0e4, 0.35, 1.0e4, 0.25, 1.0e-3;
  ES::VXd plastic(1);
  plastic << 1.01;
  fixture.parameters = std::make_shared<MaterialState>(
    elastic, plastic);

  KoiterShellFormulation formulation;
  auto material = TestUtils::makeMaterialBinding(
    fixture.asset,
    std::make_shared<KoiterStVKDefinition>(),
    std::make_shared<ShellPlasticity1Definition>(),
    fixture.parameters);
  fixture.assembler = std::make_unique<DeformationModelAssembler>(
    fixture.asset->mesh(), material.binding, formulation, false);
  fixture.absolutePositions = fixture.assembler->getRestDofs();
  for (int i = 0; i < fixture.absolutePositions.size(); i++)
    fixture.absolutePositions[i] += 0.003 * std::sin(0.9 * i + 0.4);
  return fixture;
}

TEST(DeformationModelAssembler, NonlinearMaterialGradientMatchesFD)
{
  Fixture fixture = makeFixture();
  auto &assembler = *fixture.assembler;
  const ES::VXd z = fixture.parameters->plasticValues();
  const ES::VXd elastic = fixture.parameters->elasticValues();
  auto view = [&](const ES::VXd &trial) {
    return fixture.parameters->withValues(
      std::span<const double>(elastic.data(), elastic.size()),
      std::span<const double>(trial.data(), trial.size()));
  };

  ES::VXd gradient(6);
  assembler.compute_dE_dp(
    constSpan(fixture.absolutePositions), view(z), gradient);
  constexpr double h = 1e-6;
  ES::VXd fdGradient(6);
  for (int col = 0; col < 6; col++) {
    ES::VXd zp = z;
    ES::VXd zm = z;
    zp[col] += h;
    zm[col] -= h;
    fdGradient[col] = (assembler.compute_E(constSpan(fixture.absolutePositions), view(zp)) -
                        assembler.compute_E(constSpan(fixture.absolutePositions), view(zm))) /
      (2.0 * h);
  }

  EXPECT_LT(
    (gradient - fdGradient).norm() / std::max(1.0, gradient.norm()),
    2e-6);
}

TEST(DeformationModelAssembler, PlasticMaterialVJPMatchesFD)
{
  Fixture fixture = makeFixture();
  auto &assembler = *fixture.assembler;
  const ES::VXd z = fixture.parameters->plasticValues();
  const ES::VXd elastic = fixture.parameters->elasticValues();
  auto view = [&](const ES::VXd &trial) {
    return fixture.parameters->withValues(
      std::span<const double>(elastic.data(), elastic.size()),
      std::span<const double>(trial.data(), trial.size()));
  };

  ES::VXd adjoint(assembler.getNumDOFs());
  for (int i = 0; i < adjoint.size(); ++i)
    adjoint[i] = 0.03 * std::cos(0.4 * i + 0.2);
  ES::VXd directVJP = ES::VXd::Zero(z.size());
  assembler.computePlasticMaterialVJP(
    constSpan(fixture.absolutePositions), constSpan(adjoint), view(z),
    std::span<double>(directVJP.data(), directVJP.size()));
  ES::VXd fd(z.size());
  constexpr double h = 1e-6;
  for (int col = 0; col < z.size(); col++) {
    ES::VXd zp = z;
    ES::VXd zm = z;
    zp[col] += h;
    zm[col] -= h;
    ES::VXd gp = ES::VXd::Zero(assembler.getNumDOFs());
    ES::VXd gm = ES::VXd::Zero(assembler.getNumDOFs());
    assembler.compute_dE_dx(
      constSpan(fixture.absolutePositions), view(zp), gp);
    assembler.compute_dE_dx(
      constSpan(fixture.absolutePositions), view(zm), gm);
    fd[col] = adjoint.dot(gp - gm) / (2.0 * h);
  }
  EXPECT_LT(
    (directVJP - fd).norm() /
      std::max(1.0, directVJP.norm()),
    2e-5);
}

TEST(DeformationModelAssembler, ElasticMaterialVJPMatchesFD)
{
  Fixture fixture = makeNonlinearShellFixture();
  auto &assembler = *fixture.assembler;
  ES::VXd adjoint(assembler.getNumDOFs());
  for (int i = 0; i < adjoint.size(); ++i)
    adjoint[i] = 0.02 * std::sin(0.6 * i + 0.1);
  ES::VXd directVJP = ES::VXd::Zero(
    fixture.parameters->elasticValues().size());
  assembler.computeElasticMaterialVJP(
    constSpan(fixture.absolutePositions), constSpan(adjoint),
    fixture.parameters->view(),
    std::span<double>(directVJP.data(), directVJP.size()));

  const ES::VXd elastic = fixture.parameters->elasticValues();
  const ES::VXd plastic = fixture.parameters->plasticValues();
  constexpr double h = 1e-6;
  ES::VXd fd(elastic.size());
  for (int col = 0; col < elastic.size(); ++col) {
    ES::VXd ep = elastic;
    ES::VXd em = elastic;
    ep[col] += h;
    em[col] -= h;
    ES::VXd gp = ES::VXd::Zero(assembler.getNumDOFs());
    ES::VXd gm = ES::VXd::Zero(assembler.getNumDOFs());
    assembler.compute_dE_dx(
      constSpan(fixture.absolutePositions),
      fixture.parameters->withValues(constSpan(ep), constSpan(plastic)), gp);
    assembler.compute_dE_dx(
      constSpan(fixture.absolutePositions),
      fixture.parameters->withValues(constSpan(em), constSpan(plastic)), gm);
    fd[col] = adjoint.dot(gp - gm) / (2.0 * h);
  }
  EXPECT_LT((directVJP - fd).norm() / std::max(1.0, directVJP.norm()), 3e-5);
}

TEST(DeformationModelAssembler, AcceptsAnyStateWithMatchingLengths)
{
  Fixture a = makeFixture();
  Fixture b = makeFixture();
  EXPECT_NO_THROW(
    a.assembler->compute_E(
      constSpan(a.absolutePositions), b.parameters->view()));

  const MaterialState wrongLength(
    a.parameters->elasticValues(), ES::VXd::Zero(5));
  EXPECT_THROW(
    a.assembler->compute_E(
      constSpan(a.absolutePositions), wrongLength.view()),
    std::invalid_argument);
}

TEST(DeformationModelAssembler, UnsupportedMaximumStrainThrowsWithElementContext)
{
  Fixture fixture = makeNonlinearShellFixture();
  ES::VXd strains = ES::VXd::Zero(1);

  try {
    fixture.assembler->computeMaxStrains(
      constSpan(fixture.absolutePositions),
      fixture.parameters->view(), mutableSpan(strains));
    FAIL() << "Expected unsupported maximum strain to throw.";
  }
  catch (const UnsupportedDeformationDiagnosticError &e) {
    EXPECT_NE(std::string(e.what()).find("Element 0"), std::string::npos);
    EXPECT_NE(std::string(e.what()).find("Maximum strain"), std::string::npos);
  }
}

TEST(DeformationModelAssembler, VolumetricDiagnosticsProduceFiniteValues)
{
  Fixture fixture = makeFixture();
  ES::VXd stresses = ES::VXd::Zero(1);
  ES::VXd strains = ES::VXd::Zero(1);

  fixture.assembler->computeVonMisesStresses(
    constSpan(fixture.absolutePositions),
    fixture.parameters->view(), mutableSpan(stresses));
  fixture.assembler->computeMaxStrains(
    constSpan(fixture.absolutePositions),
    fixture.parameters->view(), mutableSpan(strains));

  EXPECT_TRUE(std::isfinite(stresses[0]));
  EXPECT_TRUE(std::isfinite(strains[0]));
}

TEST(DeformationModelAssembler, IndependentOwnersEvaluateConcurrentlyWithoutInterference)
{
  Fixture a = makeFixture();
  Fixture b = makeFixture();
  const ES::VXd bBefore = b.parameters->plasticValues();
  const double expectedA = a.assembler->compute_E(
    constSpan(a.absolutePositions), a.parameters->view());
  const double expectedB = b.assembler->compute_E(
    constSpan(b.absolutePositions), b.parameters->view());

  ES::VXd changedA = a.parameters->plasticValues();
  changedA[0] += 0.01;
  const MaterialState changedStateA =
    a.parameters->withPlasticValues(
      std::span<const double>(changedA.data(), changedA.size()));
  EXPECT_TRUE(b.parameters->plasticValues().isApprox(bBefore, 0.0));
  const double changedExpectedA = a.assembler->compute_E(
    constSpan(a.absolutePositions), changedStateA.view());

  auto evalA = std::async(std::launch::async, [&]() {
    double value = 0.0;
    for (int i = 0; i < 20; i++) {
      value = a.assembler->compute_E(
        constSpan(a.absolutePositions), changedStateA.view());
    }
    return value;
  });
  auto evalB = std::async(std::launch::async, [&]() {
    double value = 0.0;
    for (int i = 0; i < 20; i++) {
      value = b.assembler->compute_E(
        constSpan(b.absolutePositions), b.parameters->view());
    }
    return value;
  });

  EXPECT_DOUBLE_EQ(evalA.get(), changedExpectedA);
  EXPECT_NE(changedExpectedA, expectedA);
  EXPECT_DOUBLE_EQ(evalB.get(), expectedB);
  EXPECT_TRUE(b.parameters->plasticValues().isApprox(bBefore, 0.0));
}

TEST(PrescribedPrincipleStressConstraintFunctions, BindsImmutableMaterialState)
{
  const double vertices[] = {
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3 };
  auto mesh = std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    4, vertices, 1, 4, elementVertices,
    SimulationMeshType::TET));
  auto asset = TestUtils::makeENuAsset(mesh, 1200.0, 0.4);

  auto parameters = std::make_shared<MaterialState>(
    ES::VXd(), ES::V3d::Ones());

  TetLinearFormulation formulation;
  auto material = TestUtils::makeMaterialBinding(
    asset,
    std::make_shared<StableNeoDefinition>(),
    std::make_shared<VolumetricPlasticity3Definition>(),
    parameters);
  auto assembler = std::make_unique<DeformationModelAssembler>(
    asset->mesh(), material.binding, formulation, false);

  const int elementID = 0;
  PrescribedPrincipleStressConstraintFunctions constraints(
    12, 0, std::span<const int>(&elementID, 1), *assembler, *parameters);
  constraints.setXToPosFunc(
    [](const ES::V3d &value, int, ES::V3d &position) {
      position = value;
    });
  const ES::V3d targetStress(0.3, 0.2, 0.1);
  constraints.setTargetPHat(std::span<const double>(targetStress.data(), 3));

  ES::V12d x;
  x << 0.0, 0.0, 0.0,
    1.15, 0.0, 0.0,
    0.0, 0.87, 0.0,
    0.0, 0.0, 1.22;
  ES::V3d initialConstraint;
  constraints.func(x, initialConstraint);

  ES::V3d changedPlastic(1.08, 0.94, 1.03);
  const MaterialState changedState =
    parameters->withPlasticValues(
      std::span<const double>(changedPlastic.data(), changedPlastic.size()));
  PrescribedPrincipleStressConstraintFunctions changedConstraints(
    12, 0, std::span<const int>(&elementID, 1), *assembler, changedState);
  changedConstraints.setXToPosFunc(
    [](const ES::V3d &value, int, ES::V3d &position) {
      position = value;
    });
  changedConstraints.setTargetPHat(
    std::span<const double>(targetStress.data(), 3));
  ES::V3d changedConstraint;
  changedConstraints.func(x, changedConstraint);
  EXPECT_FALSE(changedConstraint.isApprox(initialConstraint, 1e-12));
  EXPECT_TRUE(changedConstraint.allFinite());

  ES::V12d force;
  constraints.computeForceFromTargetPHat(x, force);
  EXPECT_TRUE(force.allFinite());

  const double traction = constraints.computeSurfaceNormalTractionFromElement(
    x, ES::V3d::UnitX(), elementID);
  EXPECT_TRUE(std::isfinite(traction));

  ES::SpMatD jacobian;
  constraints.createJacobian(jacobian);
  constraints.jacobian(x, jacobian);
  EXPECT_TRUE(ES::MXd(jacobian).allFinite());

  ES::SpMatD hessian;
  constraints.hessianAlloc(hessian);
  std::fill(
    hessian.valuePtr(),
    hessian.valuePtr() + hessian.nonZeros(),
    0.0);
  constraints.hessianInPlace(
    x, ES::V3d(0.2, -0.1, 0.3), hessian);
  EXPECT_TRUE(ES::MXd(hessian).allFinite());
}

}  // namespace
