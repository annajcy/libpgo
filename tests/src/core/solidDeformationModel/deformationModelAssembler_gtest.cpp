#include <gtest/gtest.h>
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "material/plastic/plasticModel3D3DOF.h"
#include "material/plastic/plasticModel3D6DOF.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"

#include "constraints/prescribedPrincipleStressConstraintFunctions.h"
#include "deformation/deformationModelAssembler.h"
#include "deformation/deformationModelManager.h"
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

template <typename Derived>
std::span<const double> constSpan(const Eigen::MatrixBase<Derived> &values)
{
  return std::span<const double>(values.derived().data(),
                                static_cast<size_t>(values.size()));
}

template <typename Derived>
std::span<double> mutableSpan(Eigen::MatrixBase<Derived> &values)
{
  return std::span<double>(values.derived().data(),
                           static_cast<size_t>(values.size()));
}

class SquareEvaluator final : public DifferentiableMaterialChannelMapping
{
public:
  explicit SquareEvaluator(int size): size_(size) {}

  int numInputs() const override { return size_; }
  int numChannels() const override { return size_; }
  bool isAffine() const override { return false; }

  void evaluate(
    int, int quadrature, std::span<const double> z,
    std::span<double> p) const override
  {
    const double scale = 1.0 + 0.02 * quadrature;
    for (int i = 0; i < size_; i++)
      p[i] = scale * z[i] * z[i];
  }

  void evaluateJacobian(
    int, int quadrature, std::span<const double> z,
    ES::RefMatXd output) const override
  {
    const double scale = 1.0 + 0.02 * quadrature;
    output.setZero();
    for (int i = 0; i < size_; i++)
      output(i, i) = 2.0 * scale * z[i];
  }

  void evaluateHessians(
    int, int quadrature, std::span<const double>,
    std::span<ES::MXd> channelHessians) const override
  {
    const double scale = 1.0 + 0.02 * quadrature;
    for (ES::MXd &hessian : channelHessians)
      hessian.setZero();
    for (int i = 0; i < size_; i++)
      channelHessians[static_cast<std::size_t>(i)](i, i) = 2.0 * scale;
  }

private:
  int size_;
};

class ThresholdThrowingSquareEvaluator final : public DifferentiableMaterialChannelMapping
{
public:
  ThresholdThrowingSquareEvaluator(int size, double threshold):
    size_(size), threshold_(threshold)
  {
  }

  int numInputs() const override { return size_; }
  int numChannels() const override { return size_; }
  bool isAffine() const override { return false; }

  void evaluate(
    int, int quadrature, std::span<const double> z,
    std::span<double> p) const override
  {
    if (!z.empty() && z.front() > threshold_)
      throw std::runtime_error("intentional mapping failure");
    const double scale = 1.0 + 0.02 * quadrature;
    for (int i = 0; i < size_; i++)
      p[i] = scale * z[i] * z[i];
  }

  void evaluateJacobian(
    int, int quadrature, std::span<const double> z,
    ES::RefMatXd output) const override
  {
    const double scale = 1.0 + 0.02 * quadrature;
    output.setZero();
    for (int i = 0; i < size_; i++)
      output(i, i) = 2.0 * scale * z[i];
  }

  void evaluateHessians(
    int, int quadrature, std::span<const double>,
    std::span<ES::MXd> channelHessians) const override
  {
    const double scale = 1.0 + 0.02 * quadrature;
    for (ES::MXd &hessian : channelHessians)
      hessian.setZero();
    for (int i = 0; i < size_; i++)
      channelHessians[static_cast<std::size_t>(i)](i, i) = 2.0 * scale;
  }

private:
  int size_;
  double threshold_;
};

struct Fixture
{
  std::shared_ptr<const SimulationImportResult> asset;
  std::shared_ptr<const SimulationMesh> mesh;
  std::shared_ptr<MaterialState> parameters;
  std::unique_ptr<DeformationModelAssembler> assembler;
  ES::VXd absolutePositions;
};

Fixture makeFixture(
  std::shared_ptr<const DifferentiableMaterialChannelMapping> plasticEvaluator = nullptr)
{
  const double vertices[] = {
    0, 0, 0,
    1, 0, 0,
    1, 1, 0,
    0, 1, 0,
    0, 0, 1,
    1, 0, 1,
    1, 1, 1,
    0, 1, 1,
  };
  const int elementVertices[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  Fixture fixture;
  fixture.mesh = std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    8, vertices, 1, 8, elementVertices,
    SimulationMeshType::CUBIC));
  fixture.asset = TestUtils::makeENuAsset(
    fixture.mesh, 1200.0, 0.45);

  ES::VXd z(6);
  z << std::sqrt(1.01), std::sqrt(0.004), std::sqrt(0.003),
    std::sqrt(0.995), std::sqrt(0.005), std::sqrt(1.008);
  auto elasticBlock = std::make_shared<const OptimizableParameterField>(
    ParameterInputSchema{},
    std::make_shared<ElementwiseParameterLayout>(1, 0),
    std::make_shared<IdentityMaterialChannelMapping>(0));
  auto plasticBlock = std::make_shared<const OptimizableParameterField>(
    ParameterInputSchema({ "Fxx", "Fxy", "Fxz", "Fyy", "Fyz", "Fzz" }),
    std::make_shared<ElementwiseParameterLayout>(1, 6),
    plasticEvaluator ? std::move(plasticEvaluator) :
                     std::make_shared<SquareEvaluator>(6));
  fixture.parameters = std::make_shared<MaterialState>(
    ES::VXd(), z);

  CubicLinearFormulation formulation;
  auto assignment = TestUtils::makeMaterialAssignment(
    fixture.asset,
    std::make_shared<StableNeoDefinition>(),
    std::make_shared<VolumetricPlasticity6Definition>(),
    fixture.parameters, nullptr, elasticBlock, plasticBlock);
  auto manager = std::make_shared<DeformationModelManager>(
    std::move(assignment), formulation, false);
  fixture.assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation,
    std::move(elasticBlock), std::move(plasticBlock));
  fixture.absolutePositions = fixture.assembler->getRestDofs();
  for (int i = 0; i < fixture.absolutePositions.size(); i++)
    fixture.absolutePositions[i] += 0.004 * std::sin(0.7 * i + 0.2);
  return fixture;
}

Fixture makeNonlinearShellFixture()
{
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
  };
  const int triangles[] = { 0, 1, 2 };
  pgo::Mesh::TriMeshGeo surfaceMesh(3, vertices, 1, triangles);
  Fixture fixture;
  fixture.asset = TestUtils::shareAsset(
    loadShellMesh(surfaceMesh),
    TestUtils::uniformImportedMaterialCatalog(
      surfaceMesh.numTriangles(), {"E", "nu", "h", "J"},
      {1000.0, 0.45, 1e-3, 10000.0}, "shell"));
  fixture.mesh = fixture.asset->mesh();

  ES::VXd elastic(5);
  elastic << std::sqrt(2.0e4), std::sqrt(0.35),
    std::sqrt(1.0e4), std::sqrt(0.25), std::sqrt(1.0e-3);
  ES::VXd plastic(1);
  plastic << std::sqrt(1.01);
  auto elasticBlock = std::make_shared<const OptimizableParameterField>(
    ParameterInputSchema(
      { "E_membrane", "nu_membrane", "E_bending", "nu_bending", "thickness" }),
    std::make_shared<ConstantParameterLayout>(1, 5),
    std::make_shared<SquareEvaluator>(5));
  auto plasticBlock = std::make_shared<const OptimizableParameterField>(
    ParameterInputSchema({ "stretch" }),
    std::make_shared<ConstantParameterLayout>(1, 1),
    std::make_shared<SquareEvaluator>(1));
  fixture.parameters = std::make_shared<MaterialState>(
    elastic, plastic);

  KoiterShellFormulation formulation;
  auto assignment = TestUtils::makeMaterialAssignment(
    fixture.asset,
    std::make_shared<KoiterStVKDefinition>(),
    std::make_shared<ShellPlasticity1Definition>(),
    fixture.parameters, nullptr, elasticBlock, plasticBlock);
  auto manager = std::make_shared<DeformationModelManager>(
    std::move(assignment), formulation, false);
  fixture.assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation,
    std::move(elasticBlock), std::move(plasticBlock));
  fixture.absolutePositions = fixture.assembler->getRestDofs();
  for (int i = 0; i < fixture.absolutePositions.size(); i++)
    fixture.absolutePositions[i] += 0.003 * std::sin(0.9 * i + 0.4);
  return fixture;
}

TEST(DeformationModelAssembler, NonlinearEvaluatorGradientAndHessianMatchFD)
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
  ES::SpMatD hessian = assembler.d2E_dp2_template();
  assembler.compute_d2E_dp2(
    constSpan(fixture.absolutePositions), view(z), hessian);

  constexpr double h = 1e-6;
  ES::VXd fdGradient(6);
  ES::MXd fdHessian(6, 6);
  for (int col = 0; col < 6; col++) {
    ES::VXd zp = z;
    ES::VXd zm = z;
    zp[col] += h;
    zm[col] -= h;
    fdGradient[col] = (
      assembler.compute_E(constSpan(fixture.absolutePositions), view(zp)) -
      assembler.compute_E(constSpan(fixture.absolutePositions), view(zm))) /
      (2.0 * h);

    ES::VXd gp(6), gm(6);
    assembler.compute_dE_dp(
      constSpan(fixture.absolutePositions), view(zp), gp);
    assembler.compute_dE_dp(
      constSpan(fixture.absolutePositions), view(zm), gm);
    fdHessian.col(col) = (gp - gm) / (2.0 * h);
  }

  EXPECT_LT(
    (gradient - fdGradient).norm() / std::max(1.0, gradient.norm()),
    2e-6);
  EXPECT_LT(
    (ES::MXd(hessian) - fdHessian).norm() /
      std::max(1.0, ES::MXd(hessian).norm()),
    2e-5);
}

TEST(DeformationModelAssembler, NonlinearMixedDisplacementDerivativeMatchesFD)
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

  ES::SpMatD mixed = assembler.d2E_dudp_template();
  assembler.compute_d2E_dudp(
    constSpan(fixture.absolutePositions), view(z), mixed);
  ES::VXd adjoint(assembler.getNumDOFs());
  for (int i = 0; i < adjoint.size(); ++i)
    adjoint[i] = 0.03 * std::cos(0.4 * i + 0.2);
  ES::VXd directVJP = ES::VXd::Zero(z.size());
  assembler.computePlasticMaterialVJP(
    constSpan(fixture.absolutePositions), constSpan(adjoint), view(z),
    std::span<double>(directVJP.data(), directVJP.size()));
  EXPECT_TRUE(directVJP.isApprox(mixed.transpose() * adjoint, 1e-11));
  ES::MXd fd(mixed.rows(), mixed.cols());
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
    fd.col(col) = (gp - gm) / (2.0 * h);
  }
  EXPECT_LT(
    (ES::MXd(mixed) - fd).norm() /
      std::max(1.0, ES::MXd(mixed).norm()),
    2e-5);
}

TEST(DeformationModelAssembler, NonlinearElasticPlasticMixedHessianMatchesFD)
{
  Fixture fixture = makeNonlinearShellFixture();
  auto &assembler = *fixture.assembler;
  const ES::VXd elastic = fixture.parameters->elasticValues();
  const ES::VXd plastic = fixture.parameters->plasticValues();
  auto view = [&](const ES::VXd &trialElastic) {
    return fixture.parameters->withValues(
      std::span<const double>(trialElastic.data(), trialElastic.size()),
      std::span<const double>(plastic.data(), plastic.size()));
  };

  ES::SpMatD mixed = assembler.d2E_dpde_template();
  assembler.compute_d2E_dpde(
    constSpan(fixture.absolutePositions), view(elastic), mixed);

  constexpr double h = 1e-6;
  ES::MXd fd(mixed.rows(), mixed.cols());
  for (int col = 0; col < elastic.size(); col++) {
    ES::VXd ep = elastic;
    ES::VXd em = elastic;
    ep[col] += h;
    em[col] -= h;
    ES::VXd gp(plastic.size()), gm(plastic.size());
    assembler.compute_dE_dp(
      constSpan(fixture.absolutePositions), view(ep), gp);
    assembler.compute_dE_dp(
      constSpan(fixture.absolutePositions), view(em), gm);
    fd.col(col) = (gp - gm) / (2.0 * h);
  }

  EXPECT_LT(
    (ES::MXd(mixed) - fd).norm() /
      std::max(1.0, ES::MXd(mixed).norm()),
    3e-5);
}

TEST(DeformationModelAssembler, DirectElasticMaterialVJPMatchesMixedHessian)
{
  Fixture fixture = makeNonlinearShellFixture();
  auto &assembler = *fixture.assembler;
  ES::SpMatD mixed = assembler.d2E_dude_template();
  assembler.compute_d2E_dude(
    constSpan(fixture.absolutePositions), fixture.parameters->view(), mixed);

  ES::VXd adjoint(assembler.getNumDOFs());
  for (int i = 0; i < adjoint.size(); ++i)
    adjoint[i] = 0.02 * std::sin(0.6 * i + 0.1);
  ES::VXd directVJP = ES::VXd::Zero(
    fixture.parameters->elasticValues().size());
  assembler.computeElasticMaterialVJP(
    constSpan(fixture.absolutePositions), constSpan(adjoint),
    fixture.parameters->view(),
    std::span<double>(directVJP.data(), directVJP.size()));

  EXPECT_TRUE(directVJP.isApprox(mixed.transpose() * adjoint, 1e-11));
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

TEST(DeformationModelAssembler, EvaluatorExceptionDoesNotModifyBaseState)
{
  Fixture fixture = makeFixture(
    std::make_unique<ThresholdThrowingSquareEvaluator>(6, 1.1));
  const ES::VXd before = fixture.parameters->plasticValues();
  ES::VXd trial = before;
  trial[0] = 1.2;
  const ES::VXd elastic = fixture.parameters->elasticValues();
  const MaterialState trialState = fixture.parameters->withValues(
    std::span<const double>(elastic.data(), elastic.size()),
    std::span<const double>(trial.data(), trial.size()));

  EXPECT_THROW(
    fixture.assembler->compute_E(
      constSpan(fixture.absolutePositions), trialState.view()),
    std::runtime_error);
  EXPECT_TRUE(fixture.parameters->plasticValues().isApprox(before, 0.0));
  EXPECT_NO_THROW({
    const double baseEnergy = fixture.assembler->compute_E(
      constSpan(fixture.absolutePositions), fixture.parameters->view());
    EXPECT_TRUE(std::isfinite(baseEnergy));
  });
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
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3 };
  auto mesh = std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    4, vertices, 1, 4, elementVertices,
    SimulationMeshType::TET));
  auto asset = TestUtils::makeENuAsset(mesh, 1200.0, 0.4);

  auto elasticField = std::make_shared<const OptimizableParameterField>(
    ParameterInputSchema{},
    std::make_shared<ElementwiseParameterLayout>(1, 0),
    std::make_shared<IdentityMaterialChannelMapping>(0));
  auto plasticField = std::make_shared<const OptimizableParameterField>(
    ParameterInputSchema({ "Fx", "Fy", "Fz" }),
    std::make_shared<ElementwiseParameterLayout>(1, 3),
    std::make_shared<IdentityMaterialChannelMapping>(3));
  auto parameters = std::make_shared<MaterialState>(
    ES::VXd(), ES::V3d::Ones());

  TetLinearFormulation formulation;
  auto assignment = TestUtils::makeMaterialAssignment(
    asset,
    std::make_shared<StableNeoDefinition>(),
    std::make_shared<VolumetricPlasticity3Definition>(),
    parameters, nullptr, elasticField, plasticField);
  auto manager = std::make_shared<DeformationModelManager>(
    std::move(assignment), formulation, false);

  const int elementID = 0;
  PrescribedPrincipleStressConstraintFunctions constraints(
    12, 0, std::span<const int>(&elementID, 1), *manager, *parameters);
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
    12, 0, std::span<const int>(&elementID, 1), *manager, changedState);
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
