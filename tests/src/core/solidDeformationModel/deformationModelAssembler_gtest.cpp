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
#include "material/core/materialParameters.h"
#include "simulation/simulationMesh.h"
#include "triMeshGeo.h"

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

class SquareMapping final : public MaterialChannelMapping
{
public:
  explicit SquareMapping(int size): size_(size) {}

  int numInputDofs() const override { return size_; }
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

class ThresholdThrowingSquareMapping final : public MaterialChannelMapping
{
public:
  ThresholdThrowingSquareMapping(int size, double threshold):
    size_(size), threshold_(threshold)
  {
  }

  int numInputDofs() const override { return size_; }
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
  std::shared_ptr<const SimulationMesh> mesh;
  std::shared_ptr<MaterialParameters> parameters;
  std::unique_ptr<DeformationModelAssembler> assembler;
  ES::VXd absolutePositions;
};

Fixture makeFixture(
  std::shared_ptr<const MaterialChannelMapping> plasticMapping = nullptr)
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
  SimulationMeshENuMaterial material(1200.0, 0.45);

  Fixture fixture;
  fixture.mesh = std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    8, vertices, 1, 8, elementVertices,
    makeUniformSimulationMeshElementFieldStore(1, material),
    SimulationMeshType::CUBIC));

  ES::VXd z(6);
  z << std::sqrt(1.01), std::sqrt(0.004), std::sqrt(0.003),
    std::sqrt(0.995), std::sqrt(0.005), std::sqrt(1.008);
  auto elasticBlock = MaterialParameterField::create(
    {},
    std::make_shared<ElementwiseParameterDofLayout>(1, 0),
    std::make_shared<IdentityMaterialChannelMapping>(0));
  auto plasticBlock = MaterialParameterField::create(
    { "Fxx", "Fxy", "Fxz", "Fyy", "Fyz", "Fzz" },
    std::make_shared<ElementwiseParameterDofLayout>(1, 6),
    plasticMapping ? std::move(plasticMapping) :
                     std::make_shared<SquareMapping>(6));
  auto space = std::make_shared<MaterialParameterSpace>(
    std::move(elasticBlock), std::move(plasticBlock));
  fixture.parameters = std::make_shared<MaterialParameters>(
    space, ES::VXd(), z);

  CubicLinearFormulation formulation;
  auto manager = std::make_shared<DeformationModelManager>(
    fixture.mesh,
    std::make_shared<StableNeoConfig>(),
    std::make_shared<VolumetricPlasticity6Config>(),
    formulation, 0);
  fixture.assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation, std::move(space));
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
  SimulationMeshENuhMaterial material(1000.0, 0.45, 1e-3);

  Fixture fixture;
  fixture.mesh = std::shared_ptr<const SimulationMesh>(
    loadShellMesh(surfaceMesh, material).release());

  ES::VXd elastic(5);
  elastic << std::sqrt(2.0e4), std::sqrt(0.35),
    std::sqrt(1.0e4), std::sqrt(0.25), std::sqrt(1.0e-3);
  ES::VXd plastic(1);
  plastic << std::sqrt(1.01);
  auto elasticBlock = MaterialParameterField::create(
    { "E_membrane", "nu_membrane", "E_bending", "nu_bending", "thickness" },
    std::make_shared<ConstantParameterDofLayout>(1, 5),
    std::make_shared<SquareMapping>(5));
  auto plasticBlock = MaterialParameterField::create(
    { "scale" },
    std::make_shared<ConstantParameterDofLayout>(1, 1),
    std::make_shared<SquareMapping>(1));
  auto space = std::make_shared<MaterialParameterSpace>(
    std::move(elasticBlock), std::move(plasticBlock));
  fixture.parameters = std::make_shared<MaterialParameters>(
    space, elastic, plastic);

  KoiterShellFormulation formulation;
  auto manager = std::make_shared<DeformationModelManager>(
    fixture.mesh,
    std::make_shared<KoiterStVKConfig>(),
    std::make_shared<ShellPlasticity1Config>(),
    formulation, 0);
  fixture.assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation, std::move(space));
  fixture.absolutePositions = fixture.assembler->getRestDofs();
  for (int i = 0; i < fixture.absolutePositions.size(); i++)
    fixture.absolutePositions[i] += 0.003 * std::sin(0.9 * i + 0.4);
  return fixture;
}

TEST(DeformationModelAssembler, NonlinearMappingGradientAndHessianMatchFD)
{
  Fixture fixture = makeFixture();
  auto &assembler = *fixture.assembler;
  const ES::VXd z = fixture.parameters->plasticSnapshot();
  const ES::VXd elastic = fixture.parameters->elasticSnapshot();
  auto view = [&](const ES::VXd &trial) {
    return fixture.parameters->snapshot().withValues(
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
      assembler.computeEnergy(constSpan(fixture.absolutePositions), view(zp)) -
      assembler.computeEnergy(constSpan(fixture.absolutePositions), view(zm))) /
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
  const ES::VXd z = fixture.parameters->plasticSnapshot();
  const ES::VXd elastic = fixture.parameters->elasticSnapshot();
  auto view = [&](const ES::VXd &trial) {
    return fixture.parameters->snapshot().withValues(
      std::span<const double>(elastic.data(), elastic.size()),
      std::span<const double>(trial.data(), trial.size()));
  };

  ES::SpMatD mixed = assembler.d2E_dudp_template();
  assembler.compute_d2E_dudp(
    constSpan(fixture.absolutePositions), view(z), mixed);
  ES::MXd fd(mixed.rows(), mixed.cols());
  constexpr double h = 1e-6;
  for (int col = 0; col < z.size(); col++) {
    ES::VXd zp = z;
    ES::VXd zm = z;
    zp[col] += h;
    zm[col] -= h;
    ES::VXd gp = ES::VXd::Zero(assembler.getNumDOFs());
    ES::VXd gm = ES::VXd::Zero(assembler.getNumDOFs());
    assembler.computeGradient(
      constSpan(fixture.absolutePositions), view(zp), gp);
    assembler.computeGradient(
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
  const ES::VXd elastic = fixture.parameters->elasticSnapshot();
  const ES::VXd plastic = fixture.parameters->plasticSnapshot();
  auto view = [&](const ES::VXd &trialElastic) {
    return fixture.parameters->snapshot().withValues(
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

TEST(DeformationModelAssembler, RejectsStateFromDifferentSpace)
{
  Fixture a = makeFixture();
  Fixture b = makeFixture();
  EXPECT_THROW(
    a.assembler->computeEnergy(
      constSpan(a.absolutePositions), b.parameters->snapshot().view()),
    std::invalid_argument);
}

TEST(DeformationModelAssembler, MappingExceptionDoesNotModifyCommittedState)
{
  Fixture fixture = makeFixture(
    std::make_unique<ThresholdThrowingSquareMapping>(6, 1.1));
  const ES::VXd before = fixture.parameters->plasticSnapshot();
  ES::VXd trial = before;
  trial[0] = 1.2;
  const ES::VXd elastic = fixture.parameters->elasticSnapshot();
  const MaterialParameterEvaluationView trialView =
    fixture.parameters->snapshot().withValues(
      std::span<const double>(elastic.data(), elastic.size()),
      std::span<const double>(trial.data(), trial.size()));

  EXPECT_THROW(
    fixture.assembler->computeEnergy(
      constSpan(fixture.absolutePositions), trialView),
    std::runtime_error);
  EXPECT_TRUE(fixture.parameters->plasticSnapshot().isApprox(before, 0.0));
  EXPECT_NO_THROW({
    const double committedEnergy = fixture.assembler->computeEnergy(
      constSpan(fixture.absolutePositions), fixture.parameters->snapshot().view());
    EXPECT_TRUE(std::isfinite(committedEnergy));
  });
}

TEST(DeformationModelAssembler, UnsupportedMaximumStrainThrowsWithElementContext)
{
  Fixture fixture = makeNonlinearShellFixture();
  ES::VXd strains = ES::VXd::Zero(1);

  try {
    fixture.assembler->computeMaxStrains(
      constSpan(fixture.absolutePositions),
      fixture.parameters->snapshot().view(), mutableSpan(strains));
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
    fixture.parameters->snapshot().view(), mutableSpan(stresses));
  fixture.assembler->computeMaxStrains(
    constSpan(fixture.absolutePositions),
    fixture.parameters->snapshot().view(), mutableSpan(strains));

  EXPECT_TRUE(std::isfinite(stresses[0]));
  EXPECT_TRUE(std::isfinite(strains[0]));
}

TEST(DeformationModelAssembler, IndependentOwnersEvaluateConcurrentlyWithoutInterference)
{
  Fixture a = makeFixture();
  Fixture b = makeFixture();
  const ES::VXd bBefore = b.parameters->plasticSnapshot();
  const double expectedA = a.assembler->computeEnergy(
    constSpan(a.absolutePositions), a.parameters->snapshot().view());
  const double expectedB = b.assembler->computeEnergy(
    constSpan(b.absolutePositions), b.parameters->snapshot().view());

  ES::VXd changedA = a.parameters->plasticSnapshot();
  changedA[0] += 0.01;
  a.parameters->setPlasticValues(changedA);
  EXPECT_TRUE(b.parameters->plasticSnapshot().isApprox(bBefore, 0.0));
  const double changedExpectedA = a.assembler->computeEnergy(
    constSpan(a.absolutePositions), a.parameters->snapshot().view());

  auto evalA = std::async(std::launch::async, [&]() {
    double value = 0.0;
    for (int i = 0; i < 20; i++) {
      value = a.assembler->computeEnergy(
        constSpan(a.absolutePositions), a.parameters->snapshot().view());
    }
    return value;
  });
  auto evalB = std::async(std::launch::async, [&]() {
    double value = 0.0;
    for (int i = 0; i < 20; i++) {
      value = b.assembler->computeEnergy(
        constSpan(b.absolutePositions), b.parameters->snapshot().view());
    }
    return value;
  });

  EXPECT_DOUBLE_EQ(evalA.get(), changedExpectedA);
  EXPECT_NE(changedExpectedA, expectedA);
  EXPECT_DOUBLE_EQ(evalB.get(), expectedB);
  EXPECT_TRUE(b.parameters->plasticSnapshot().isApprox(bBefore, 0.0));
}

TEST(PrescribedPrincipleStressConstraintFunctions, UsesCommittedMaterialParameters)
{
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3 };
  SimulationMeshENuMaterial material(1200.0, 0.4);
  auto mesh = std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    4, vertices, 1, 4, elementVertices,
    makeUniformSimulationMeshElementFieldStore(1, material),
    SimulationMeshType::TET));

  auto elasticField = MaterialParameterField::create(
    {},
    std::make_shared<ElementwiseParameterDofLayout>(1, 0),
    std::make_shared<IdentityMaterialChannelMapping>(0));
  auto plasticField = MaterialParameterField::create(
    { "stretch_x", "stretch_y", "stretch_z" },
    std::make_shared<ElementwiseParameterDofLayout>(1, 3),
    std::make_shared<IdentityMaterialChannelMapping>(3));
  auto space = std::make_shared<MaterialParameterSpace>(
    std::move(elasticField), std::move(plasticField));
  auto parameters = std::make_shared<MaterialParameters>(
    space, ES::VXd(), ES::V3d::Ones());

  TetLinearFormulation formulation;
  auto manager = std::make_shared<DeformationModelManager>(
    mesh,
    std::make_shared<StableNeoConfig>(),
    std::make_shared<VolumetricPlasticity3Config>(),
    formulation, false);

  const int elementID = 0;
  PrescribedPrincipleStressConstraintFunctions constraints(
    12, 0, std::span<const int>(&elementID, 1), *manager, parameters);
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
  parameters->setPlasticValues(changedPlastic);
  ES::V3d changedConstraint;
  constraints.func(x, changedConstraint);
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
