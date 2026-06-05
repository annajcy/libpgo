// Patch + finite-difference tests for the regular-grid tricubic Hermite hex formulation.
//
// A single axis-aligned unit cube is an affine element, so the synthesized rest Hermite field is
// exactly affine and the rest deformation gradient is exactly I. That lets us pin:
//   * rest state              -> zero energy / gradient
//   * rigid translation       -> zero energy   (F = I)
//   * rigid rotation          -> zero energy   (F = R, frame-invariant material)
//   * any affine deformation  -> identical energy to the trilinear hex (both have constant F = A)
//   * gradient(u) == d func/du, hessian(u) == d gradient/du   (enforceSPD = 0 -> true derivative)

#include <gtest/gtest.h>

#include "deformationModelAssembler.h"
#include "deformationModelEnergy.h"
#include "deformationModelManager.h"
#include "deformationModelState.h"
#include "plasticModel3DDeformationGradient.h"
#include "simulationMesh.h"
#include "formulations/formulation.h"
#include "pgoLogging.h"
#include "EigenSupport.h"

#include <tbb/global_control.h>

#include <cmath>
#include <memory>

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

constexpr double kFiniteDifferenceStep = 1e-6;
constexpr int kExactDerivativeEnforceSpd = 0;

class ScopedSerialTbb
{
public:
  ScopedSerialTbb(): control_(tbb::global_control::max_allowed_parallelism, 1) {}

private:
  tbb::global_control control_;
};

struct EnergyCase
{
  std::shared_ptr<const SimulationMesh> meshOwner;
  std::shared_ptr<DeformationModelState> state;
  std::unique_ptr<DeformationModelEnergy> energy;
  int numDOFs = 0;
};

void setVolumetricPlasticIdentity(DeformationModelEnergy &energy, DeformationModelState &state)
{
  const auto &assembler = energy.assembler();
  const auto *plasticModel = dynamic_cast<const PlasticModel3DDeformationGradient *>(
    assembler.getDeformationModelManager().getDeformationModel(0)->getPlasticModel());
  ASSERT_NE(plasticModel, nullptr);
  const int nele = assembler.getDeformationModelManager().getMesh()->getNumElements();
  const int npp = assembler.getNumPlasticParams();
  ES::VXd plastic(static_cast<Eigen::Index>(npp) * nele);
  ES::M3d identity = ES::M3d::Identity();
  for (int ei = 0; ei < nele; ei++)
    plasticModel->toParam(identity.data(), plastic.data() + ei * npp);
  state.setPlasticValues(plastic);
}

// Single axis-aligned unit cube (CUBIC element, 8 vertices in HexTrilinearBasis corner order).
std::shared_ptr<const SimulationMesh> makeUnitCubeMesh()
{
  pgo::Logging::init();
  static const double vertices[] = {
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0
  };
  static const int elementVertices[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  static const int elementMaterialIndices[] = { 0 };
  static SimulationMeshENuMaterial baseMaterial(1200.0, 0.45);
  static const SimulationMeshMaterial *materials[] = { &baseMaterial };
  return std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    8, vertices, 1, 8, elementVertices, elementMaterialIndices, 1, materials, SimulationMeshType::CUBIC));
}

template<class FormulationT>
EnergyCase makeCubeCase(const FormulationT &formulation)
{
  EnergyCase c;
  c.meshOwner = makeUnitCubeMesh();
  c.state = DeformationModelState::create(
    c.meshOwner, DeformationModelElasticMaterial::STABLE_NEO, ElasticFieldInit{},
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, PlasticFieldInit{});
  auto manager = std::make_unique<DeformationModelManager>(
    c.state, formulation, kExactDerivativeEnforceSpd, nullptr, nullptr);
  auto assembler = std::make_unique<DeformationModelAssembler>(std::move(manager), nullptr);
  c.energy = std::make_unique<DeformationModelEnergy>(std::move(assembler), 0, false);
  c.numDOFs = c.energy->getNumDOFs();
  setVolumetricPlasticIdentity(*c.energy, *c.state);
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

TEST(TricubicHermiteFormulationGTest, NumDofsIs24PerVertex)
{
  auto c = makeCubeCase(TricubicHermiteFormulation{});
  EXPECT_EQ(c.numDOFs, c.meshOwner->getNumVertices() * 24);
  EXPECT_EQ(c.numDOFs, 192);
}

TEST(TricubicHermiteFormulationGTest, RestStateHasZeroEnergyAndGradient)
{
  auto c = makeCubeCase(TricubicHermiteFormulation{});
  ES::VXd u = ES::VXd::Zero(c.numDOFs);
  EXPECT_NEAR(c.energy->func(u), 0.0, 1e-9);
  ES::VXd g(c.numDOFs);
  c.energy->gradient(u, g);
  EXPECT_LT(g.norm(), 1e-7);
}

TEST(TricubicHermiteFormulationGTest, RigidTranslationHasZeroEnergy)
{
  auto c = makeCubeCase(TricubicHermiteFormulation{});
  ES::VXd u = hermiteAffineDisplacement(
    c.energy->getRestPosition(), ES::M3d::Identity(), ES::V3d(0.3, -0.7, 1.1));
  EXPECT_NEAR(c.energy->func(u), 0.0, 1e-9);
  ES::VXd g(c.numDOFs);
  c.energy->gradient(u, g);
  EXPECT_LT(g.norm(), 1e-7);
}

TEST(TricubicHermiteFormulationGTest, RigidRotationHasZeroEnergy)
{
  auto c = makeCubeCase(TricubicHermiteFormulation{});
  ES::M3d R = Eigen::AngleAxisd(0.4, ES::V3d(0.3, 0.8, 0.5).normalized()).toRotationMatrix();
  ES::VXd u = hermiteAffineDisplacement(c.energy->getRestPosition(), R, ES::V3d::Zero());
  EXPECT_NEAR(c.energy->func(u), 0.0, 1e-6);
  ES::VXd g(c.numDOFs);
  c.energy->gradient(u, g);
  EXPECT_LT(g.norm(), 1e-4);
}

// A constant deformation gradient produces the same total energy regardless of element technology.
TEST(TricubicHermiteFormulationGTest, AffineEnergyMatchesTrilinear)
{
  auto hermite = makeCubeCase(TricubicHermiteFormulation{});
  auto trilinear = makeCubeCase(LinearCubicFormulation{});

  ES::M3d A;
  A << 1.05, 0.03, 0.0,
       0.0, 0.98, 0.02,
       0.01, 0.0, 1.03;
  ES::V3d t(0.05, -0.02, 0.01);

  ES::VXd uH = hermiteAffineDisplacement(hermite.energy->getRestPosition(), A, t);
  ES::VXd uT = trilinearAffineDisplacement(trilinear.energy->getRestPosition(), A, t);

  double eH = hermite.energy->func(uH);
  double eT = trilinear.energy->func(uT);
  EXPECT_GT(eT, 0.0);
  EXPECT_NEAR(eH, eT, 1e-7 * std::max(1.0, std::abs(eT)));
}

TEST(TricubicHermiteFormulationGTest, GradientMatchesFiniteDifference)
{
  auto c = makeCubeCase(TricubicHermiteFormulation{});
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
      kFiniteDifferenceStep);
  }

  const double err = (fd - analytic).norm() / std::max(1.0, analytic.norm());
  EXPECT_LT(err, 1e-6) << "gradient disagrees with FD of func (rel err " << err << ")";
  EXPECT_GT(analytic.norm(), 0.0);
}

TEST(TricubicHermiteFormulationGTest, HessianMatchesFiniteDifference)
{
  auto c = makeCubeCase(TricubicHermiteFormulation{});
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
      kFiniteDifferenceStep);
  }

  const double err = (fd - analytic).norm() / std::max(1.0, analytic.norm());
  EXPECT_LT(err, 1e-6) << "Hessian disagrees with FD of gradient (rel err " << err << ")";
  EXPECT_GT(analytic.norm(), 0.0);
}
