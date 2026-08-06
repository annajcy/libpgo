// Patch + finite-difference tests for the regular-grid tricubic Hermite hex formulation.
//
// A single axis-aligned unit cube is an affine element, so the synthesized rest Hermite field is
// exactly affine and the rest deformation gradient is exactly I. That lets us pin:
//   * rest state              -> zero energy / gradient
//   * rigid translation       -> zero energy   (F = I)
//   * rigid rotation          -> zero energy   (F = R, frame-invariant material)
//   * any affine deformation  -> identical energy to the trilinear hex (both have constant F = A)
//   * gradient(u) == d func/du, hessian(u) == d gradient/du   (projectHessianPSD = false -> true derivative)

#include <gtest/gtest.h>
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "material/plastic/plasticModel3D3DOF.h"
#include "material/plastic/plasticModel3D6DOF.h"
#include "material/plastic/plasticModel2DFundamentalFormsUniformStretch.h"

#include "deformation/deformationModelAssembler.h"
#include "energy/deformationEnergyOperator.h"
#include "../materialTestUtils.h"
#include "material/plastic/plasticModel3DDeformationGradient.h"
#include "simulation/simulationMesh.h"
#include "formulations/formulation/formulations.h"
#include "pgoLogging.h"
#include "EigenSupport.h"

#include <tbb/global_control.h>

#include <cmath>
#include <memory>
#include <span>

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

constexpr double kFiniteDifferenceStep = 1e-6;
constexpr bool kExactDerivativeProjectHessianPSD = false;

double fdStep(double base) { return kFiniteDifferenceStep * std::max(1.0, std::abs(base)); }

class ScopedSerialTbb
{
public:
  ScopedSerialTbb(): control_(tbb::global_control::max_allowed_parallelism, 1) {}

private:
  tbb::global_control control_;
};

struct EnergyCase
{
  std::shared_ptr<const TestUtils::TestAsset> asset;
  std::shared_ptr<const SimulationMesh> meshOwner;
  std::unique_ptr<DeformationPotentialEnergy> energy;
  int numDOFs = 0;
};

// Single axis-aligned unit cube (CUBIC element, 8 vertices in CubicLinearShapeFunction corner order).
std::shared_ptr<const SimulationMesh> makeUnitCubeMesh()
{
  pgo::Logging::init();
  static const double vertices[] = {
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0
  };
  static const int elementVertices[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  return std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    8, vertices, 1, 8, elementVertices,
    SimulationMeshType::CUBIC));
}

template<class FormulationT>
EnergyCase makeCubeCase(const FormulationT &formulation, int offset = 0)
{
  EnergyCase c;
  c.meshOwner = makeUnitCubeMesh();
  c.asset = TestUtils::makeENuAsset(c.meshOwner, 1200.0, 0.45);
  auto parameters = TestUtils::makeDefaultMaterialState(
    *c.asset, *std::make_shared<StableNeoDefinition>(),
    *std::make_shared<VolumetricPlasticity6Definition>());
  auto material = TestUtils::makeMaterialBinding(
    c.asset, std::make_shared<StableNeoDefinition>(),
    std::make_shared<VolumetricPlasticity6Definition>(), parameters);
  DeformationModelOptions options;
  options.projectHessianPSD = kExactDerivativeProjectHessianPSD;
  options.dofOffset = offset;
  auto energyOperator = std::make_shared<DeformationEnergyOperator>(
    *c.asset->mesh(), *material.binding, formulation, options);
  c.energy = std::make_unique<DeformationPotentialEnergy>(
    std::move(energyOperator), *material.state);
  c.numDOFs = c.energy->getNumDOFs();
  return c;
}

// Apply an affine map x = A X + t to the Hermite DOF vector: the value mode of each vertex maps as
// A*pos + t, every derivative mode maps as A*mode (no translation). Returns the displacement u.
ES::VXd hermiteAffineDisplacement(const ES::VXd &rest, const ES::M3d &A, const ES::V3d &t)
{
  ES::VXd u = ES::VXd::Zero(rest.size());
  const int nvtx = static_cast<int>(rest.size()) / 24;
  for (int v = 0; v < nvtx; v++) {
    for (int m = 0; m < 8; m++) {
      ES::V3d r = rest.segment<3>(v * 24 + m * 3);
      ES::V3d d = A * r + (m == 0 ? t : ES::V3d::Zero());
      u.segment<3>(v * 24 + m * 3) = d - r;
    }
  }
  return u;
}

ES::VXd trilinearAffineDisplacement(const ES::VXd &rest, const ES::M3d &A, const ES::V3d &t)
{
  ES::VXd u = ES::VXd::Zero(rest.size());
  const int nvtx = static_cast<int>(rest.size()) / 3;
  for (int v = 0; v < nvtx; v++) {
    ES::V3d p = rest.segment<3>(v * 3);
    u.segment<3>(v * 3) = (A * p + t) - p;
  }
  return u;
}

template<class Eval>
double fivePointScalar(Eval eval, double h)
{
  return (-eval(2.0 * h) + 8.0 * eval(h) - 8.0 * eval(-h) + eval(-2.0 * h)) / (12.0 * h);
}

template<class Eval>
ES::VXd fivePointVector(Eval eval, double h)
{
  return (-eval(2.0 * h) + 8.0 * eval(h) - 8.0 * eval(-h) + eval(-2.0 * h)) / (12.0 * h);
}

// Small smooth displacement over all 192 Hermite DOFs so the gradient/Hessian are nontrivial.
ES::VXd makeSmoothHermiteDisplacement(int n)
{
  ES::VXd u(n);
  for (int i = 0; i < n; i++)
    u[i] = 2e-3 * std::sin(0.7 * i + 0.2) + 1e-3 * std::cos(0.31 * i + 0.9);
  return u;
}
}  // namespace

TEST(CubicTricubicHermiteFormulationGTest, NumDofsIs24PerVertex)
{
  auto c = makeCubeCase(CubicTricubicHermiteFormulation{});
  EXPECT_EQ(c.numDOFs, c.meshOwner->getNumVertices() * 24);
  EXPECT_EQ(c.numDOFs, 192);
}

TEST(CubicTricubicHermiteFormulationGTest, RestStateHasZeroEnergyAndGradient)
{
  auto c = makeCubeCase(CubicTricubicHermiteFormulation{});
  ES::VXd u = ES::VXd::Zero(c.numDOFs);
  EXPECT_NEAR(c.energy->func(u), 0.0, 1e-9);
  ES::VXd g(c.numDOFs);
  c.energy->gradient(u, g);
  EXPECT_LT(g.norm(), 1e-7);
}

TEST(CubicTricubicHermiteFormulationGTest, RigidTranslationHasZeroEnergy)
{
  auto c = makeCubeCase(CubicTricubicHermiteFormulation{});
  ES::VXd u = hermiteAffineDisplacement(
    c.energy->getRestDofs(), ES::M3d::Identity(), ES::V3d(0.3, -0.7, 1.1));
  EXPECT_NEAR(c.energy->func(u), 0.0, 1e-9);
  ES::VXd g(c.numDOFs);
  c.energy->gradient(u, g);
  EXPECT_LT(g.norm(), 1e-7);
}

TEST(CubicTricubicHermiteFormulationGTest, RigidRotationHasZeroEnergy)
{
  auto c = makeCubeCase(CubicTricubicHermiteFormulation{});
  ES::M3d R = Eigen::AngleAxisd(0.4, ES::V3d(0.3, 0.8, 0.5).normalized()).toRotationMatrix();
  ES::VXd u = hermiteAffineDisplacement(c.energy->getRestDofs(), R, ES::V3d::Zero());
  EXPECT_NEAR(c.energy->func(u), 0.0, 1e-6);
  ES::VXd g(c.numDOFs);
  c.energy->gradient(u, g);
  EXPECT_LT(g.norm(), 1e-4);
}

// A constant deformation gradient produces the same total energy regardless of element technology.
TEST(CubicTricubicHermiteFormulationGTest, AffineEnergyMatchesTrilinear)
{
  auto hermite = makeCubeCase(CubicTricubicHermiteFormulation{});
  auto trilinear = makeCubeCase(CubicLinearFormulation{});

  ES::M3d A;
  A << 1.05, 0.03, 0.0,
       0.0, 0.98, 0.02,
       0.01, 0.0, 1.03;
  ES::V3d t(0.05, -0.02, 0.01);

  ES::VXd uH = hermiteAffineDisplacement(hermite.energy->getRestDofs(), A, t);
  ES::VXd uT = trilinearAffineDisplacement(trilinear.energy->getRestDofs(), A, t);

  double eH = hermite.energy->func(uH);
  double eT = trilinear.energy->func(uT);
  EXPECT_GT(eT, 0.0);
  EXPECT_NEAR(eH, eT, 1e-7 * std::max(1.0, std::abs(eT)));
}

TEST(CubicTricubicHermiteFormulationGTest, GradientMatchesFiniteDifference)
{
  auto c = makeCubeCase(CubicTricubicHermiteFormulation{});
  ES::VXd u = makeSmoothHermiteDisplacement(c.numDOFs);

  ES::VXd analytic(c.numDOFs);
  c.energy->gradient(u, analytic);

  ScopedSerialTbb serial;
  ES::VXd fd(c.numDOFs);
  for (int i = 0; i < c.numDOFs; i++) {
    fd[i] = fivePointScalar([&](double delta) {
      ES::VXd up = u;
      up[i] += delta;
      return c.energy->func(up);
    },
      fdStep(u[i]));
  }

  const double err = (fd - analytic).norm() / std::max(1.0, analytic.norm());
  EXPECT_LT(err, 1e-6) << "gradient disagrees with FD of func (rel err " << err << ")";
  EXPECT_GT(analytic.norm(), 0.0);
}

TEST(CubicTricubicHermiteFormulationGTest, GradientMatchesFiniteDifferenceWithOffset)
{
  constexpr int kOffset = 12;
  auto c = makeCubeCase(CubicTricubicHermiteFormulation{}, kOffset);
  std::vector<int> dofs;
  c.energy->getDOFs(dofs);
  ASSERT_EQ(dofs.size(), static_cast<std::size_t>(c.numDOFs));
  EXPECT_EQ(dofs.front(), kOffset);
  EXPECT_EQ(dofs.back(), kOffset + c.numDOFs - 1);

  ES::VXd u = makeSmoothHermiteDisplacement(c.numDOFs);

  ES::VXd analytic(c.numDOFs);
  c.energy->gradient(u, analytic);

  ScopedSerialTbb serial;
  ES::VXd fd(c.numDOFs);
  for (int i = 0; i < c.numDOFs; i++) {
    fd[i] = fivePointScalar([&](double delta) {
      ES::VXd up = u;
      up[i] += delta;
      return c.energy->func(up);
    },
      fdStep(u[i]));
  }

  const double err = (fd - analytic).norm() / std::max(1.0, analytic.norm());
  EXPECT_LT(err, 1e-6) << "gradient with offset disagrees with FD of func (rel err " << err << ")";
  EXPECT_GT(analytic.norm(), 0.0);
}

TEST(CubicTricubicHermiteFormulationGTest, HessianMatchesFiniteDifference)
{
  auto c = makeCubeCase(CubicTricubicHermiteFormulation{});
  ES::VXd u = makeSmoothHermiteDisplacement(c.numDOFs);

  ES::SpMatD H;
  c.energy->hessianAlloc(H);
  c.energy->hessianInPlace(u, H);
  ES::MXd analytic(H);

  ScopedSerialTbb serial;
  ES::MXd fd(c.numDOFs, c.numDOFs);
  for (int i = 0; i < c.numDOFs; i++) {
    fd.col(i) = fivePointVector([&](double delta) {
      ES::VXd up = u;
      up[i] += delta;
      ES::VXd g(c.numDOFs);
      c.energy->gradient(up, g);
      return g;
    },
      fdStep(u[i]));
  }

  const double err = (fd - analytic).norm() / std::max(1.0, analytic.norm());
  EXPECT_LT(err, 1e-5) << "Hessian disagrees with FD of gradient (rel err " << err << ")";
  EXPECT_GT(analytic.norm(), 0.0);
}

TEST(CubicTricubicHermiteFormulationGTest, HessianMatchesFiniteDifferenceWithOffset)
{
  constexpr int kOffset = 12;
  auto c = makeCubeCase(CubicTricubicHermiteFormulation{}, kOffset);
  ES::VXd u = makeSmoothHermiteDisplacement(c.numDOFs);

  ES::SpMatD H;
  c.energy->hessianAlloc(H);
  c.energy->hessianInPlace(u, H);
  ES::MXd analytic(H);

  ScopedSerialTbb serial;
  ES::MXd fd(c.numDOFs, c.numDOFs);
  for (int i = 0; i < c.numDOFs; i++) {
    fd.col(i) = fivePointVector([&](double delta) {
      ES::VXd up = u;
      up[i] += delta;
      ES::VXd g(c.numDOFs);
      c.energy->gradient(up, g);
      return g;
    },
      fdStep(u[i]));
  }

  const double err = (fd - analytic).norm() / std::max(1.0, analytic.norm());
  EXPECT_LT(err, 1e-5) << "Hessian with offset disagrees with FD of gradient (rel err " << err << ")";
  EXPECT_GT(analytic.norm(), 0.0);
}

// Two adjacent unit cubes sharing a face: 12 vertices, 2 elements.
// Tests that shared-vertex DOFs interact correctly through the DofLayout scatter.
std::shared_ptr<const SimulationMesh> makeTwoCubeMesh()
{
  pgo::Logging::init();
  // Cube 0: vertices 0-7, Cube 1 (shifted by +1 in X): uses vertices 1,5,6,2,9,10,11,?
  // Layout: v0=(0,0,0), v1=(1,0,0), v2=(1,1,0), v3=(0,1,0),
  //         v4=(0,0,1), v5=(1,0,1), v6=(1,1,1), v7=(0,1,1),
  //         v8=(2,0,0), v9=(2,1,0), v10=(2,0,1), v11=(2,1,1)
  // Shared face: v1,v5,v6,v2 (x=1 plane)
  static const double vertices[] = {
    0.0, 0.0, 0.0,  1.0, 0.0, 0.0,  1.0, 1.0, 0.0,  0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,  1.0, 0.0, 1.0,  1.0, 1.0, 1.0,  0.0, 1.0, 1.0,
    2.0, 0.0, 0.0,  2.0, 1.0, 0.0,  2.0, 0.0, 1.0,  2.0, 1.0, 1.0,
  };
  static const int elementVertices[] = {
    0, 1, 2, 3, 4, 5, 6, 7,     // cube 0
    1, 8, 9, 2, 5, 10, 11, 6,    // cube 1 (shared: v1,v2,v5,v6)
  };
  return std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    12, vertices, 2, 8, elementVertices,
    SimulationMeshType::CUBIC));
}

template<class FormulationT>
EnergyCase makeTwoCubeCase(const FormulationT &formulation)
{
  EnergyCase c;
  c.meshOwner = makeTwoCubeMesh();
  c.asset = TestUtils::makeENuAsset(c.meshOwner, 1200.0, 0.45);
  auto parameters = TestUtils::makeDefaultMaterialState(
    *c.asset, *std::make_shared<StableNeoDefinition>(),
    *std::make_shared<VolumetricPlasticity6Definition>());
  auto material = TestUtils::makeMaterialBinding(
    c.asset, std::make_shared<StableNeoDefinition>(),
    std::make_shared<VolumetricPlasticity6Definition>(), parameters);
  DeformationModelOptions options;
  options.projectHessianPSD = kExactDerivativeProjectHessianPSD;
  auto energyOperator = std::make_shared<DeformationEnergyOperator>(
    *c.asset->mesh(), *material.binding, formulation, options);
  c.energy = std::make_unique<DeformationPotentialEnergy>(
    std::move(energyOperator), *material.state);
  c.numDOFs = c.energy->getNumDOFs();
  return c;
}

TEST(CubicTricubicHermiteFormulationGTest, TwoElementNumDofs)
{
  auto c = makeTwoCubeCase(CubicTricubicHermiteFormulation{});
  EXPECT_EQ(c.numDOFs, 12 * 24);
  EXPECT_EQ(c.numDOFs, 288);
}

TEST(CubicTricubicHermiteFormulationGTest, TwoElementHessianTemplateIsSharedByVertices)
{
  auto c = makeTwoCubeCase(CubicTricubicHermiteFormulation{});
  ES::SpMatD H;
  c.energy->hessianAlloc(H);
  EXPECT_EQ(H.rows(), c.numDOFs);
  EXPECT_EQ(H.cols(), c.numDOFs);
  EXPECT_LT(H.nonZeros(), 2 * 192 * 192);
  EXPECT_GT(H.nonZeros(), 192 * 192);
}

TEST(CubicTricubicHermiteFormulationGTest, TwoElementGradientMatchesFiniteDifference)
{
  auto c = makeTwoCubeCase(CubicTricubicHermiteFormulation{});
  ES::VXd u = makeSmoothHermiteDisplacement(c.numDOFs);

  ES::VXd analytic(c.numDOFs);
  c.energy->gradient(u, analytic);

  ScopedSerialTbb serial;
  ES::VXd fd(c.numDOFs);
  for (int i = 0; i < c.numDOFs; i++) {
    fd[i] = fivePointScalar([&](double delta) {
      ES::VXd up = u;
      up[i] += delta;
      return c.energy->func(up);
    },
      fdStep(u[i]));
  }

  const double err = (fd - analytic).norm() / std::max(1.0, analytic.norm());
  EXPECT_LT(err, 1e-6) << "two-element gradient disagrees with FD of func (rel err " << err << ")";
  EXPECT_GT(analytic.norm(), 0.0);
}

TEST(CubicTricubicHermiteFormulationGTest, TwoElementHessianMatchesFiniteDifference)
{
  auto c = makeTwoCubeCase(CubicTricubicHermiteFormulation{});
  ES::VXd u = makeSmoothHermiteDisplacement(c.numDOFs);

  ES::SpMatD H;
  c.energy->hessianAlloc(H);
  c.energy->hessianInPlace(u, H);
  ES::MXd analytic(H);

  ScopedSerialTbb serial;
  ES::MXd fd(c.numDOFs, c.numDOFs);
  for (int i = 0; i < c.numDOFs; i++) {
    fd.col(i) = fivePointVector([&](double delta) {
      ES::VXd up = u;
      up[i] += delta;
      ES::VXd g(c.numDOFs);
      c.energy->gradient(up, g);
      return g;
    },
      fdStep(u[i]));
  }

  const double err = (fd - analytic).norm() / std::max(1.0, analytic.norm());
  EXPECT_LT(err, 1e-5) << "two-element Hessian disagrees with FD of gradient (rel err " << err << ")";
  EXPECT_GT(analytic.norm(), 0.0);
}
