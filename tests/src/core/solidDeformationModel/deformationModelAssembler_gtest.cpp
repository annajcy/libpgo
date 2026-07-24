#include <gtest/gtest.h>

#include "deformation/deformationModelAssembler.h"
#include "deformation/deformationModelManager.h"
#include "formulations/formulation/formulations.h"
#include "material/fields/materialParameters.h"
#include "simulation/simulationMesh.h"
#include "triMeshGeo.h"

#include <cmath>
#include <future>
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

class SquareMapping final : public ParameterFieldMapping
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
    double *output) const override
  {
    const double scale = 1.0 + 0.02 * quadrature;
    std::fill(output, output + size_ * size_, 0.0);
    for (int i = 0; i < size_; i++)
      output[i * size_ + i] = 2.0 * scale * z[i];
  }

  void evaluateHessians(
    int, int quadrature, std::span<const double>,
    double *output) const override
  {
    const double scale = 1.0 + 0.02 * quadrature;
    std::fill(output, output + size_ * size_ * size_, 0.0);
    for (int i = 0; i < size_; i++)
      output[i * size_ * size_ + i * size_ + i] = 2.0 * scale;
  }

private:
  int size_;
};

class ThresholdThrowingSquareMapping final : public ParameterFieldMapping
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
    double *output) const override
  {
    const double scale = 1.0 + 0.02 * quadrature;
    std::fill(output, output + size_ * size_, 0.0);
    for (int i = 0; i < size_; i++)
      output[i * size_ + i] = 2.0 * scale * z[i];
  }

  void evaluateHessians(
    int, int quadrature, std::span<const double>,
    double *output) const override
  {
    const double scale = 1.0 + 0.02 * quadrature;
    std::fill(output, output + size_ * size_ * size_, 0.0);
    for (int i = 0; i < size_; i++)
      output[i * size_ * size_ + i * size_ + i] = 2.0 * scale;
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
  std::unique_ptr<const ParameterFieldMapping> plasticMapping = nullptr)
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
  const int materialIndices[] = { 0 };
  SimulationMeshENuMaterial material(1200.0, 0.45);
  const SimulationMeshMaterial *materials[] = { &material };

  Fixture fixture;
  fixture.mesh = std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    8, vertices, 1, 8, elementVertices, materialIndices, 1, materials,
    SimulationMeshType::CUBIC));

  ES::VXd z(6);
  z << std::sqrt(1.01), std::sqrt(0.004), std::sqrt(0.003),
    std::sqrt(0.995), std::sqrt(0.005), std::sqrt(1.008);
  MaterialParameterBlock elasticBlock(
    MaterialParameterBlockKind::ELASTIC, "stable_neo", {},
    std::make_unique<ElementwiseParameterDofLayout>(1, 0),
    std::make_unique<IdentityParameterFieldMapping>(0));
  MaterialParameterBlock plasticBlock(
    MaterialParameterBlockKind::PLASTIC, "volumetric_dof6",
    { "Fxx", "Fxy", "Fxz", "Fyy", "Fyz", "Fzz" },
    std::make_unique<ElementwiseParameterDofLayout>(1, 6),
    plasticMapping ? std::move(plasticMapping) :
                     std::make_unique<SquareMapping>(6));
  auto space = std::make_shared<MaterialParameterSpace>(
    std::move(elasticBlock), std::move(plasticBlock));
  fixture.parameters = std::make_shared<MaterialParameters>(
    space, ES::VXd(), z);

  CubicLinearFormulation formulation;
  auto manager = std::make_shared<DeformationModelManager>(
    fixture.mesh,
    DeformationModelElasticMaterial::STABLE_NEO,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    formulation, 0);
  fixture.assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation, std::move(space));
  fixture.absolutePositions = fixture.assembler->getRestPosition();
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
    loadShellMesh(surfaceMesh, &material).release());

  ES::VXd elastic(5);
  elastic << std::sqrt(2.0e4), std::sqrt(0.35),
    std::sqrt(1.0e4), std::sqrt(0.25), std::sqrt(1.0e-3);
  ES::VXd plastic(1);
  plastic << std::sqrt(1.01);
  MaterialParameterBlock elasticBlock(
    MaterialParameterBlockKind::ELASTIC, "koiter_stvk",
    { "E_membrane", "nu_membrane", "E_bending", "nu_bending", "thickness" },
    std::make_unique<ConstantParameterDofLayout>(1, 5),
    std::make_unique<SquareMapping>(5));
  MaterialParameterBlock plasticBlock(
    MaterialParameterBlockKind::PLASTIC, "shell_ff_dof1",
    { "scale" },
    std::make_unique<ConstantParameterDofLayout>(1, 1),
    std::make_unique<SquareMapping>(1));
  auto space = std::make_shared<MaterialParameterSpace>(
    std::move(elasticBlock), std::move(plasticBlock));
  fixture.parameters = std::make_shared<MaterialParameters>(
    space, elastic, plastic);

  KoiterShellFormulation formulation;
  auto manager = std::make_shared<DeformationModelManager>(
    fixture.mesh,
    DeformationModelElasticMaterial::KOITER_STVK,
    DeformationModelPlasticMaterial::SHELL_FF_DOF1,
    formulation, 0);
  fixture.assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation, std::move(space));
  fixture.absolutePositions = fixture.assembler->getRestPosition();
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
    return fixture.parameters->space()->makeStateView(
      std::span<const double>(elastic.data(), elastic.size()),
      std::span<const double>(trial.data(), trial.size()));
  };

  ES::VXd gradient(6);
  assembler.computePlasticGradient(
    fixture.absolutePositions.data(), view(z), gradient.data());
  ES::SpMatD hessian = assembler.getPlasticHessianTemplate();
  assembler.computePlasticHessian(
    fixture.absolutePositions.data(), view(z), hessian);

  constexpr double h = 1e-6;
  ES::VXd fdGradient(6);
  ES::MXd fdHessian(6, 6);
  for (int col = 0; col < 6; col++) {
    ES::VXd zp = z;
    ES::VXd zm = z;
    zp[col] += h;
    zm[col] -= h;
    fdGradient[col] = (
      assembler.computeEnergy(fixture.absolutePositions.data(), view(zp)) -
      assembler.computeEnergy(fixture.absolutePositions.data(), view(zm))) /
      (2.0 * h);

    ES::VXd gp(6), gm(6);
    assembler.computePlasticGradient(
      fixture.absolutePositions.data(), view(zp), gp.data());
    assembler.computePlasticGradient(
      fixture.absolutePositions.data(), view(zm), gm.data());
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
    return fixture.parameters->space()->makeStateView(
      std::span<const double>(elastic.data(), elastic.size()),
      std::span<const double>(trial.data(), trial.size()));
  };

  ES::SpMatD mixed = assembler.get_dfda_Template();
  assembler.compute_df_da(
    fixture.absolutePositions.data(), view(z), mixed);
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
      fixture.absolutePositions.data(), view(zp), gp.data());
    assembler.computeGradient(
      fixture.absolutePositions.data(), view(zm), gm.data());
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
    return fixture.parameters->space()->makeStateView(
      std::span<const double>(trialElastic.data(), trialElastic.size()),
      std::span<const double>(plastic.data(), plastic.size()));
  };

  ES::SpMatD mixed = assembler.getPlasticElasticHessianTemplate();
  assembler.computePlasticElasticHessian(
    fixture.absolutePositions.data(), view(elastic), mixed);

  constexpr double h = 1e-6;
  ES::MXd fd(mixed.rows(), mixed.cols());
  for (int col = 0; col < elastic.size(); col++) {
    ES::VXd ep = elastic;
    ES::VXd em = elastic;
    ep[col] += h;
    em[col] -= h;
    ES::VXd gp(plastic.size()), gm(plastic.size());
    assembler.computePlasticGradient(
      fixture.absolutePositions.data(), view(ep), gp.data());
    assembler.computePlasticGradient(
      fixture.absolutePositions.data(), view(em), gm.data());
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
      a.absolutePositions.data(), b.parameters->committedView()),
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
  const MaterialStateView trialView =
    fixture.parameters->space()->makeStateView(
      std::span<const double>(elastic.data(), elastic.size()),
      std::span<const double>(trial.data(), trial.size()));

  EXPECT_THROW(
    fixture.assembler->computeEnergy(
      fixture.absolutePositions.data(), trialView),
    std::runtime_error);
  EXPECT_TRUE(fixture.parameters->plasticSnapshot().isApprox(before, 0.0));
  EXPECT_NO_THROW({
    const double committedEnergy = fixture.assembler->computeEnergy(
      fixture.absolutePositions.data(), fixture.parameters->committedView());
    EXPECT_TRUE(std::isfinite(committedEnergy));
  });
}

TEST(DeformationModelAssembler, IndependentOwnersEvaluateConcurrentlyWithoutInterference)
{
  Fixture a = makeFixture();
  Fixture b = makeFixture();
  const ES::VXd bBefore = b.parameters->plasticSnapshot();
  const double expectedA = a.assembler->computeEnergy(
    a.absolutePositions.data(), a.parameters->committedView());
  const double expectedB = b.assembler->computeEnergy(
    b.absolutePositions.data(), b.parameters->committedView());

  ES::VXd changedA = a.parameters->plasticSnapshot();
  changedA[0] += 0.01;
  a.parameters->setPlasticValues(changedA);
  EXPECT_TRUE(b.parameters->plasticSnapshot().isApprox(bBefore, 0.0));
  const double changedExpectedA = a.assembler->computeEnergy(
    a.absolutePositions.data(), a.parameters->committedView());

  auto evalA = std::async(std::launch::async, [&]() {
    double value = 0.0;
    for (int i = 0; i < 20; i++) {
      value = a.assembler->computeEnergy(
        a.absolutePositions.data(), a.parameters->committedView());
    }
    return value;
  });
  auto evalB = std::async(std::launch::async, [&]() {
    double value = 0.0;
    for (int i = 0; i < 20; i++) {
      value = b.assembler->computeEnergy(
        b.absolutePositions.data(), b.parameters->committedView());
    }
    return value;
  });

  EXPECT_DOUBLE_EQ(evalA.get(), changedExpectedA);
  EXPECT_NE(changedExpectedA, expectedA);
  EXPECT_DOUBLE_EQ(evalB.get(), expectedB);
  EXPECT_TRUE(b.parameters->plasticSnapshot().isApprox(bBefore, 0.0));
}

}  // namespace
