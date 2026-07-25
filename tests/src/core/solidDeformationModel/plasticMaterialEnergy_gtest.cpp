#include <gtest/gtest.h>
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "material/plastic/plasticModel3D3DOF.h"
#include "material/plastic/plasticModel3D6DOF.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"

#include "deformation/deformationModelAssembler.h"
#include "energy/deformationModelEnergy.h"
#include "deformation/deformationModelManager.h"
#include "formulations/formulation/formulations.h"
#include "material/fields/materialParameterBuilder.h"
#include "energy/elasticMaterialEnergy.h"
#include "energy/plasticMaterialEnergy.h"
#include "simulation/simulationMesh.h"
#include "triMeshGeo.h"
#include "pgoLogging.h"
#include "energy/evaluation.h"

#include <tbb/global_control.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;
using pgo::SolidDeformationModel::DeformationModelAssembler;
using pgo::SolidDeformationModel::ElasticModelConfig;
using pgo::SolidDeformationModel::DeformationModelEnergy;
using pgo::SolidDeformationModel::DeformationModelManager;
using pgo::SolidDeformationModel::PlasticModelConfig;
using pgo::SolidDeformationModel::ElasticMaterialEnergy;
using pgo::SolidDeformationModel::PlasticMaterialEnergy;
using pgo::SolidDeformationModel::SimulationMesh;
using pgo::SolidDeformationModel::SimulationMeshENuhMaterial;
using pgo::SolidDeformationModel::SimulationMeshENuMaterial;
using pgo::SolidDeformationModel::SimulationMeshType;

constexpr double kFiniteDifferenceStep = 1e-6;
constexpr bool kExactDerivativeProjectHessianPSD = false;

class ScopedSerialTbb
{
public:
  ScopedSerialTbb(): control_(tbb::global_control::max_allowed_parallelism, 1) {}

private:
  tbb::global_control control_;
};

template<class Eval>
double fivePointScalar(Eval eval, double h)
{
  return (-eval(2.0 * h) + 8.0 * eval(h) - 8.0 * eval(-h) + eval(-2.0 * h)) / (12.0 * h);
}

template<class Eval>
ES::VXd fivePointVector(Eval eval, double h)
{
  ES::VXd gp2 = eval(2.0 * h);
  ES::VXd gp1 = eval(h);
  ES::VXd gm1 = eval(-h);
  ES::VXd gm2 = eval(-2.0 * h);
  return (-gp2 + 8.0 * gp1 - 8.0 * gm1 + gm2) / (12.0 * h);
}

std::shared_ptr<const SimulationMesh> makeSingleHexMesh()
{
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    1.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
    1.0, 0.0, 1.0,
    1.0, 1.0, 1.0,
    0.0, 1.0, 1.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  SimulationMeshENuMaterial baseMaterial(1200.0, 0.45);

  return std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    8, vertices,
    1, 8, elementVertices,
    makeUniformSimulationMeshElementFieldStore(1, baseMaterial),
    SimulationMeshType::CUBIC));
}

ES::VXd makeFixedDisplacement(int numVertices)
{
  ES::VXd u = ES::VXd::Zero(static_cast<Eigen::Index>(numVertices) * 3);
  for (int vi = 0; vi < numVertices; vi++) {
    u[vi * 3 + 0] = 5e-3 * std::sin(0.9 * vi + 0.1);
    u[vi * 3 + 1] = 4e-3 * std::cos(0.7 * vi + 0.3);
    u[vi * 3 + 2] = 3e-3 * std::sin(1.3 * vi + 0.5);
  }
  return u;
}

void expectExactlyEqual(const ES::VXd &actual, const ES::VXd &expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  EXPECT_TRUE((actual.array() == expected.array()).all())
    << "actual:   " << actual.transpose() << '\n'
    << "expected: " << expected.transpose();
}

std::shared_ptr<DeformationModelEnergy> makeDeformationEnergy(std::shared_ptr<const SimulationMesh> mesh, const ES::VXd &plasticBase)
{
  pgo::SolidDeformationModel::CubicLinearFormulation formulation;
  auto space = pgo::SolidDeformationModel::makeMaterialParameterSpace(
    *std::make_shared<StableNeoConfig>(),
    std::make_shared<pgo::SolidDeformationModel::ElementwiseParameterDofLayout>(1, 0),
    std::make_shared<pgo::SolidDeformationModel::IdentityMaterialChannelMapping>(0),
    *std::make_shared<VolumetricPlasticity6Config>(),
    std::make_shared<pgo::SolidDeformationModel::ElementwiseParameterDofLayout>(1, 6),
    std::make_shared<pgo::SolidDeformationModel::IdentityMaterialChannelMapping>(6));
  auto parameters = pgo::SolidDeformationModel::makeMaterialParameters(
    std::move(space), ES::VXd::Zero(0), plasticBase);
  auto manager = std::make_shared<DeformationModelManager>(
    mesh, std::make_shared<StableNeoConfig>(), std::make_shared<VolumetricPlasticity6Config>(),
    formulation, kExactDerivativeProjectHessianPSD);
  auto assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation, parameters->space(), nullptr);
  return std::make_shared<DeformationModelEnergy>(
    std::move(assembler), std::move(parameters), 0, false);
}

std::shared_ptr<DeformationModelEnergy> makeShellDeformationEnergy(const ES::VXd &elasticBase)
{
  constexpr int N = 4;
  std::vector<double> vertices;
  vertices.reserve(N * N * 3);
  for (int j = 0; j < N; j++)
    for (int i = 0; i < N; i++) {
      vertices.push_back(0.1 * i);
      vertices.push_back(0.1 * j);
      vertices.push_back(0.0);
    }

  auto vid = [](int i, int j) { return j * N + i; };
  std::vector<int> triangles;
  for (int j = 0; j < N - 1; j++)
    for (int i = 0; i < N - 1; i++) {
      triangles.push_back(vid(i, j));
      triangles.push_back(vid(i + 1, j));
      triangles.push_back(vid(i + 1, j + 1));
      triangles.push_back(vid(i, j));
      triangles.push_back(vid(i + 1, j + 1));
      triangles.push_back(vid(i, j + 1));
    }

  pgo::Mesh::TriMeshGeo surfaceMesh(N * N, vertices.data(),
    static_cast<int>(triangles.size() / 3), triangles.data());
  SimulationMeshENuhMaterial mat(1000.0, 0.45, 1e-3);
  std::shared_ptr<const SimulationMesh> mesh(
    pgo::SolidDeformationModel::loadShellMesh(surfaceMesh, &mat).release());

  pgo::SolidDeformationModel::KoiterShellFormulation formulation;
  auto space = pgo::SolidDeformationModel::makeMaterialParameterSpace(
    *std::make_shared<KoiterStVKConfig>(),
    std::make_shared<pgo::SolidDeformationModel::ConstantParameterDofLayout>(
      mesh->getNumElements(), 5),
    std::make_shared<pgo::SolidDeformationModel::IdentityMaterialChannelMapping>(5),
    *std::make_shared<ShellPlasticity1Config>(),
    std::make_shared<pgo::SolidDeformationModel::ElementwiseParameterDofLayout>(
      mesh->getNumElements(), 1),
    std::make_shared<pgo::SolidDeformationModel::IdentityMaterialChannelMapping>(1));
  auto parameters = pgo::SolidDeformationModel::makeMaterialParameters(
    std::move(space), elasticBase,
    ES::VXd::Constant(mesh->getNumElements(), 1.0));
  auto manager = std::make_shared<DeformationModelManager>(
    mesh, std::make_shared<KoiterStVKConfig>(), std::make_shared<ShellPlasticity1Config>(),
    formulation, kExactDerivativeProjectHessianPSD);
  auto assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation, parameters->space(), nullptr);
  return std::make_shared<DeformationModelEnergy>(
    std::move(assembler), std::move(parameters), 0, false);
}
}  // namespace

TEST(PlasticMaterialEnergyGTest, ValueMatchesDeformationEnergyAtFixedDisplacement)
{
  pgo::Logging::init();

  auto mesh = makeSingleHexMesh();
  ES::VXd plasticBase(6);
  plasticBase << 1.01, 0.004, -0.003, 0.994, 0.005, 1.008;

  auto deformationEnergy = makeDeformationEnergy(mesh, plasticBase);
  ES::VXd fixedDisplacement = makeFixedDisplacement(mesh->getNumVertices());

  PlasticMaterialEnergy plasticEnergy(deformationEnergy, fixedDisplacement);
  EXPECT_EQ(plasticEnergy.stateKind(), pgo::NonlinearOptimization::EnergyStateKind::Generic);
  EXPECT_EQ(plasticEnergy.getNumDOFs(), 6);

  std::vector<int> dofs;
  plasticEnergy.getDOFs(dofs);
  ASSERT_EQ(dofs.size(), 6u);
  for (int i = 0; i < 6; i++)
    EXPECT_EQ(dofs[i], i);

  deformationEnergy->materialParameters()->setPlasticValues(plasticBase);
  const double expected = deformationEnergy->func(fixedDisplacement);
  const double actual = plasticEnergy.func(plasticBase);
  EXPECT_NEAR(actual, expected, 1e-12 * std::max(1.0, std::abs(expected)));
}

TEST(PlasticMaterialEnergyGTest, EvaluationDoesNotModifyParentPlasticState)
{
  pgo::Logging::init();

  auto mesh = makeSingleHexMesh();
  ES::VXd committed(6);
  committed << 1.01, 0.004, -0.003, 0.994, 0.005, 1.008;

  ES::VXd trial = committed;
  trial[0] += 0.02;
  trial[3] -= 0.01;
  trial[5] += 0.015;

  auto deformationEnergy = makeDeformationEnergy(mesh, committed);
  ES::VXd fixedDisplacement = makeFixedDisplacement(mesh->getNumVertices());
  PlasticMaterialEnergy plasticEnergy(deformationEnergy, fixedDisplacement);

  auto parameters = deformationEnergy->materialParameters();
  ES::VXd grad(plasticEnergy.getNumDOFs());
  ES::SpMatD hess;
  plasticEnergy.hessianAlloc(hess);

  auto expectPreserved = [&](const char *operation, auto &&evaluate) {
    SCOPED_TRACE(operation);

    parameters->setPlasticValues(committed);
    const ES::VXd before = parameters->plasticSnapshot();

    evaluate();

    const ES::VXd after = parameters->plasticSnapshot();
    expectExactlyEqual(after, before);
  };

  expectPreserved("func", [&] {
    (void)plasticEnergy.func(trial);
  });

  expectPreserved("gradient", [&] {
    plasticEnergy.gradient(trial, grad);
  });

  expectPreserved("hessian", [&] {
    plasticEnergy.hessianInPlace(trial, hess);
  });
}

TEST(PlasticMaterialEnergyGTest, UsesElasticSnapshotCapturedAtConstruction)
{
  pgo::Logging::init();

  ES::VXd elastic(5);
  elastic << 20000.0, 0.45, 10000.0, 0.3, 1e-3;

  auto deformationEnergy = makeShellDeformationEnergy(elastic);
  ES::VXd fixedDisplacement = makeFixedDisplacement(
    deformationEnergy->assembler().getDeformationModelManager().getMesh()->getNumVertices());
  auto parameters = deformationEnergy->materialParameters();

  const ES::VXd originalElastic = parameters->elasticSnapshot();
  const ES::VXd plastic = parameters->plasticSnapshot();
  ASSERT_GT(originalElastic.size(), 0);
  ASSERT_GT(plastic.size(), 0);

  PlasticMaterialEnergy plasticEnergy(deformationEnergy, fixedDisplacement);
  const double expected = plasticEnergy.func(plastic);

  ES::VXd changedElastic = originalElastic;
  changedElastic[0] *= 1.5;
  parameters->setElasticValues(changedElastic);

  const double actual = plasticEnergy.func(plastic);

  EXPECT_NEAR(actual, expected, 1e-12 * std::max(1.0, std::abs(expected)));
}

TEST(PlasticMaterialEnergyGTest, GradientAndHessianMatchFiniteDifference)
{
  pgo::Logging::init();

  auto mesh = makeSingleHexMesh();
  ES::VXd plasticBase(6);
  plasticBase << 1.01, 0.004, -0.003, 0.994, 0.005, 1.008;

  auto deformationEnergy = makeDeformationEnergy(mesh, plasticBase);
  ES::VXd fixedDisplacement = makeFixedDisplacement(mesh->getNumVertices());
  PlasticMaterialEnergy plasticEnergy(deformationEnergy, fixedDisplacement);

  ES::VXd grad(6);
  plasticEnergy.gradient(plasticBase, grad);
  EXPECT_GT(grad.norm(), 0.0);

  ScopedSerialTbb serial;
  ES::VXd fdGrad(6);
  for (int i = 0; i < 6; i++) {
    fdGrad[i] = fivePointScalar([&](double delta) {
      ES::VXd p = plasticBase;
      p[i] += delta;
      return plasticEnergy.func(p);
    },
      kFiniteDifferenceStep);
  }
  EXPECT_LT((fdGrad - grad).norm() / std::max(1.0, grad.norm()), 1e-6);

  ES::SpMatD hess;
  plasticEnergy.hessianAlloc(hess);
  plasticEnergy.hessianInPlace(plasticBase, hess);
  ES::MXd hessDense(hess);
  ASSERT_EQ(hessDense.rows(), 6);
  ASSERT_EQ(hessDense.cols(), 6);

  ES::MXd fdHess(6, 6);
  for (int i = 0; i < 6; i++) {
    fdHess.col(i) = fivePointVector([&](double delta) {
      ES::VXd p = plasticBase;
      p[i] += delta;
      ES::VXd g(6);
      plasticEnergy.gradient(p, g);
      return g;
    },
      kFiniteDifferenceStep);
  }
  EXPECT_LT((fdHess - hessDense).norm() / std::max(1.0, hessDense.norm()), 1e-6);

  EXPECT_DOUBLE_EQ(pgo::NonlinearOptimization::evaluateValue(plasticEnergy, plasticBase), plasticEnergy.func(plasticBase));
  EXPECT_TRUE(pgo::NonlinearOptimization::evaluateGradient(plasticEnergy, plasticBase).isApprox(grad, 1e-12));
  EXPECT_EQ(pgo::NonlinearOptimization::evaluateHessian(plasticEnergy, plasticBase).rows(), 6);
}

TEST(ElasticMaterialEnergyGTest, EvaluationDoesNotModifyParentElasticState)
{
  pgo::Logging::init();

  ES::VXd committed(5);
  committed << 20000.0, 0.45, 10000.0, 0.3, 1e-3;

  ES::VXd trial = committed;
  trial[0] *= 1.1;
  trial[2] *= 0.9;
  trial[4] *= 1.2;

  auto deformationEnergy = makeShellDeformationEnergy(committed);
  ES::VXd fixedDisplacement = makeFixedDisplacement(
    deformationEnergy->assembler().getDeformationModelManager().getMesh()->getNumVertices());
  ElasticMaterialEnergy elasticEnergy(deformationEnergy, fixedDisplacement);

  auto parameters = deformationEnergy->materialParameters();
  ES::VXd grad(elasticEnergy.getNumDOFs());
  ES::SpMatD hess;
  elasticEnergy.hessianAlloc(hess);

  auto expectPreserved = [&](const char *operation, auto &&evaluate) {
    SCOPED_TRACE(operation);

    parameters->setElasticValues(committed);
    const ES::VXd before = parameters->elasticSnapshot();

    evaluate();

    const ES::VXd after = parameters->elasticSnapshot();
    expectExactlyEqual(after, before);
  };

  expectPreserved("func", [&] {
    (void)elasticEnergy.func(trial);
  });

  expectPreserved("gradient", [&] {
    elasticEnergy.gradient(trial, grad);
  });

  expectPreserved("hessian", [&] {
    elasticEnergy.hessianInPlace(trial, hess);
  });
}

TEST(ElasticMaterialEnergyGTest, UsesPlasticSnapshotCapturedAtConstruction)
{
  pgo::Logging::init();

  ES::VXd elastic(5);
  elastic << 20000.0, 0.45, 10000.0, 0.3, 1e-3;

  auto deformationEnergy = makeShellDeformationEnergy(elastic);
  ES::VXd fixedDisplacement = makeFixedDisplacement(
    deformationEnergy->assembler().getDeformationModelManager().getMesh()->getNumVertices());
  auto parameters = deformationEnergy->materialParameters();

  const ES::VXd originalPlastic = parameters->plasticSnapshot();
  ElasticMaterialEnergy elasticEnergy(deformationEnergy, fixedDisplacement);

  const double expected = elasticEnergy.func(elastic);

  ES::VXd changedPlastic = originalPlastic;
  ASSERT_GT(changedPlastic.size(), 0);
  changedPlastic.array() *= 1.05;
  parameters->setPlasticValues(changedPlastic);

  const double actual = elasticEnergy.func(elastic);

  EXPECT_NEAR(actual, expected, 1e-12 * std::max(1.0, std::abs(expected)));
}

TEST(ElasticMaterialEnergyGTest, ValueGradientAndHessianMatchFiniteDifference)
{
  pgo::Logging::init();

  ES::VXd elasticBase(5);
  elasticBase << 20000.0, 0.45, 10000.0, 0.3, 1e-3;

  auto deformationEnergy = makeShellDeformationEnergy(elasticBase);
  ES::VXd fixedDisplacement = makeFixedDisplacement(
    deformationEnergy->assembler().getDeformationModelManager().getMesh()->getNumVertices());
  ElasticMaterialEnergy elasticEnergy(deformationEnergy, fixedDisplacement);

  EXPECT_EQ(elasticEnergy.stateKind(), pgo::NonlinearOptimization::EnergyStateKind::Generic);
  EXPECT_EQ(elasticEnergy.getNumDOFs(), 5);

  std::vector<int> dofs;
  elasticEnergy.getDOFs(dofs);
  ASSERT_EQ(dofs.size(), 5u);
  for (int i = 0; i < 5; i++)
    EXPECT_EQ(dofs[i], i);

  deformationEnergy->materialParameters()->setElasticValues(elasticBase);
  const double expected = deformationEnergy->func(fixedDisplacement);
  const double actual = elasticEnergy.func(elasticBase);
  EXPECT_NEAR(actual, expected, 1e-12 * std::max(1.0, std::abs(expected)));

  ES::VXd grad(5);
  elasticEnergy.gradient(elasticBase, grad);
  EXPECT_GT(grad.norm(), 0.0);

  ScopedSerialTbb serial;
  ES::VXd fdGrad(5);
  for (int i = 0; i < 5; i++) {
    const double h = kFiniteDifferenceStep * std::max(1.0, std::abs(elasticBase[i]));
    fdGrad[i] = fivePointScalar([&](double delta) {
      ES::VXd p = elasticBase;
      p[i] += delta;
      return elasticEnergy.func(p);
    },
      h);
  }
  EXPECT_LT((fdGrad - grad).norm() / std::max(1.0, grad.norm()), 1e-5);

  ES::SpMatD hess;
  elasticEnergy.hessianAlloc(hess);
  elasticEnergy.hessianInPlace(elasticBase, hess);
  ES::MXd hessDense(hess);
  ASSERT_EQ(hessDense.rows(), 5);
  ASSERT_EQ(hessDense.cols(), 5);

  ES::MXd fdHess(5, 5);
  for (int i = 0; i < 5; i++) {
    const double h = kFiniteDifferenceStep * std::max(1.0, std::abs(elasticBase[i]));
    fdHess.col(i) = fivePointVector([&](double delta) {
      ES::VXd p = elasticBase;
      p[i] += delta;
      ES::VXd g(5);
      elasticEnergy.gradient(p, g);
      return g;
    },
      h);
  }
  EXPECT_LT((fdHess - hessDense).norm() / std::max(1.0, hessDense.norm()), 1e-5);

  EXPECT_DOUBLE_EQ(pgo::NonlinearOptimization::evaluateValue(elasticEnergy, elasticBase), elasticEnergy.func(elasticBase));
  EXPECT_TRUE(pgo::NonlinearOptimization::evaluateGradient(elasticEnergy, elasticBase).isApprox(grad, 1e-12));
  EXPECT_EQ(pgo::NonlinearOptimization::evaluateHessian(elasticEnergy, elasticBase).rows(), 5);
}
