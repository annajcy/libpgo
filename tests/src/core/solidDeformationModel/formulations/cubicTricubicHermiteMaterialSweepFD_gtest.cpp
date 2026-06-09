// Material-breadth finite-difference sweep for the tricubic Hermite hex formulation.
//
// Runs the two material-agnostic position-derivative identities
//   gradient == FD(energy)        (exercises P = dpsi/dF)
//   hessian  == FD(gradient)      (exercises dPdF), enforceSPD = 0
// across EVERY volumetric elastic material the factory supports, on a single
// tricubic-Hermite unit cube. Uses the Energy API (displacement state) so the
// full chain — Hermite rest-field synthesis, rest+u addition, DOF scatter —
// is exercised.
//
// FD steps are magnitude-scaled: h = 1e-6 * max(1, |u[i]|) per the FD test
// roadmap convention.

#include <gtest/gtest.h>

#include "deformation/deformationModelAssembler.h"
#include "energy/deformationModelEnergy.h"
#include "deformation/deformationModelManager.h"
#include "material/fields/materialParameterFieldInit.h"
#include "material/plastic/plasticModel3DDeformationGradient.h"
#include "simulation/simulationMesh.h"
#include "formulations/formulation/formulations.h"
#include "pgoLogging.h"
#include "EigenSupport.h"

#include <tbb/global_control.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace
{
namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

constexpr double kFiniteDifferenceStep = 1e-6;
constexpr int kExactDerivativeEnforceSpd = 0;

double fdStep(double base) { return kFiniteDifferenceStep * std::max(1.0, std::abs(base)); }

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
  ES::VXd gp2 = eval(2.0 * h), gp1 = eval(h), gm1 = eval(-h), gm2 = eval(-2.0 * h);
  return (-gp2 + 8.0 * gp1 - 8.0 * gm1 + gm2) / (12.0 * h);
}

// Set plastic to identity for all elements.
void setVolumetricPlasticIdentity(DeformationModelEnergy &energy)
{
  auto &assembler = energy.assembler();
  const auto &manager = assembler.getDeformationModelManager();
  const int nele = assembler.getDeformationModelManager().getMesh()->getNumElements();
  const int npp = assembler.getNumPlasticParams();
  ES::VXd plastic(static_cast<Eigen::Index>(npp) * nele);
  for (int ei = 0; ei < nele; ei++)
    manager.getDeformationModel(ei)->defaultPlasticParams(plastic.data() + ei * npp);
  assembler.setPlasticValues(plastic);
}

// Smooth nonzero displacement over all Hermite DOFs.
ES::VXd makeSmoothHermiteDisplacement(int n)
{
  ES::VXd u(n);
  for (int i = 0; i < n; i++)
    u[i] = 2e-3 * std::sin(0.7 * i + 0.2) + 1e-3 * std::cos(0.31 * i + 0.9);
  return u;
}

struct HermiteSweepCase
{
  std::shared_ptr<const SimulationMesh> meshOwner;
  std::unique_ptr<DeformationModelEnergy> energy;
  ES::VXd elementFiber, vertexFiber;
  int numDOFs = 0;
};

// Build a single-element tricubic-Hermite Energy for a given material.
// withHill: appends Hill activation parameter + fiber directions.
HermiteSweepCase makeHermiteCase(DeformationModelElasticMaterial elastic, bool withHill)
{
  HermiteSweepCase c;

  // Always create a mutable mesh first so we can optionally append the Hill material.
  static SimulationMeshENuMaterial baseMaterial(1200.0, 0.45);
  static const double vertices[] = {
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0
  };
  static const int elementVertices[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  static const int elementMaterialIndices[] = { 0 };
  const SimulationMeshMaterial *materials[] = { &baseMaterial };
  auto meshMutable = std::make_unique<SimulationMesh>(
    8, vertices, 1, 8, elementVertices, elementMaterialIndices, 1, materials, SimulationMeshType::CUBIC);

  if (withHill) {
    static SimulationMeshHillMaterial hillMaterial(2500.0, 0.35, 1.0);
    meshMutable->appendMaterialToAllElements(&hillMaterial);
  }
  c.meshOwner = std::shared_ptr<const SimulationMesh>(std::move(meshMutable));

  const int nele = c.meshOwner->getNumElements();
  const int nvtx = c.meshOwner->getNumVertices();
  c.elementFiber = ES::VXd::Zero(nele * 3);
  c.vertexFiber = ES::VXd::Zero(nvtx * 3);
  for (int ei = 0; ei < nele; ei++)
    c.elementFiber.segment<3>(ei * 3) << 1.0, 0.0, 0.0;
  for (int vi = 0; vi < nvtx; vi++)
    c.vertexFiber.segment<3>(vi * 3) << 1.0, 0.0, 0.0;

  auto elasticField = createElasticParameterField(*c.meshOwner, elastic, ElasticFieldInit{});
  auto plasticField = createPlasticParameterField(
    *c.meshOwner, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, PlasticFieldInit{});

  const double *ef = withHill ? c.elementFiber.data() : nullptr;
  const double *vf = withHill ? c.vertexFiber.data() : nullptr;
  CubicTricubicHermiteFormulation formulation;
  auto manager = std::make_shared<DeformationModelManager>(
    c.meshOwner, elastic, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    formulation, kExactDerivativeEnforceSpd, ef, vf);
  auto assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation, std::move(elasticField), std::move(plasticField), nullptr);
  c.energy = std::make_unique<DeformationModelEnergy>(std::move(assembler), 0, false);
  c.numDOFs = c.energy->getNumDOFs();
  setVolumetricPlasticIdentity(*c.energy);
  return c;
}

void checkGradientVsFDFunc(HermiteSweepCase &c, double tol, const char *name)
{
  ES::VXd u = makeSmoothHermiteDisplacement(c.numDOFs);

  ES::VXd analytic(c.numDOFs);
  c.energy->gradient(u, analytic);
  EXPECT_GT(analytic.norm(), 0.0) << name << ": gradient is zero; FD check is vacuous.";

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
  EXPECT_LT(err, tol) << name << ": gradient disagrees with FD(energy), rel err " << err << ".";
}

void checkHessianVsFDGradient(HermiteSweepCase &c, double tol, const char *name)
{
  ES::VXd u = makeSmoothHermiteDisplacement(c.numDOFs);

  ES::SpMatD H;
  c.energy->hessianAlloc(H);
  c.energy->hessianInPlace(u, H);
  ES::MXd analytic(H);
  EXPECT_GT(analytic.norm(), 0.0) << name << ": Hessian is zero; FD check is vacuous.";

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
  EXPECT_LT(err, tol) << name << ": Hessian disagrees with FD(gradient), rel err " << err << ".";
}

void runHermiteSweep(DeformationModelElasticMaterial elastic, bool withHill, const char *name,
  double gradTol = 1e-6, double hessTol = 1e-5)
{
  HermiteSweepCase c = makeHermiteCase(elastic, withHill);
  checkGradientVsFDFunc(c, gradTol, name);
  checkHessianVsFDGradient(c, hessTol, name);
}
}  // namespace

TEST(CubicTricubicHermiteMaterialSweepFDGTest, StableNeo) { runHermiteSweep(DeformationModelElasticMaterial::STABLE_NEO, false, "STABLE_NEO"); }
TEST(CubicTricubicHermiteMaterialSweepFDGTest, StVK) { runHermiteSweep(DeformationModelElasticMaterial::STVK, false, "STVK"); }
TEST(CubicTricubicHermiteMaterialSweepFDGTest, StVKVol) { runHermiteSweep(DeformationModelElasticMaterial::STVK_VOL, false, "STVK_VOL"); }
TEST(CubicTricubicHermiteMaterialSweepFDGTest, InvStVK) { runHermiteSweep(DeformationModelElasticMaterial::INV_STVK, false, "INV_STVK"); }
TEST(CubicTricubicHermiteMaterialSweepFDGTest, Volume) { runHermiteSweep(DeformationModelElasticMaterial::VOLUME, false, "VOLUME"); }
TEST(CubicTricubicHermiteMaterialSweepFDGTest, Linear) { runHermiteSweep(DeformationModelElasticMaterial::LINEAR, false, "LINEAR"); }

TEST(CubicTricubicHermiteMaterialSweepFDGTest, MooneyRivlin)
{
  // Explicit Cpq / D coefficients (a known-valid 2-term config); the (E, nu)
  // convenience constructor only seeds C(1,0)/D(0) and yields a degenerate
  // (NaN-producing) material here.
  ES::M3d Cpq;
  Cpq << 0.0, 1.0, 1.0,
         1.0, 1.0, 1.0,
         1.0, 1.0, 1.0;
  ES::V2d D;
  D << 1.0, 1.0;

  // Build a mesh with explicit Mooney-Rivlin material, then build the case manually.
  pgo::Logging::init();
  static const double vertices[] = {
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0
  };
  static const int elementVertices[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  static const int elementMaterialIndices[] = { 0 };
  SimulationMeshMooneyRivlinMaterial mooney(2, 2, Cpq.data(), D.data());
  const SimulationMeshMaterial *materials[] = { &mooney };
  auto mesh = std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    8, vertices, 1, 8, elementVertices, elementMaterialIndices, 1, materials, SimulationMeshType::CUBIC));

  HermiteSweepCase c;
  c.meshOwner = mesh;
  auto elasticField = createElasticParameterField(
    *c.meshOwner, DeformationModelElasticMaterial::MOONEY_RIVLIN, ElasticFieldInit{});
  auto plasticField = createPlasticParameterField(
    *c.meshOwner, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, PlasticFieldInit{});
  CubicTricubicHermiteFormulation formulation;
  auto manager = std::make_shared<DeformationModelManager>(
    c.meshOwner, DeformationModelElasticMaterial::MOONEY_RIVLIN,
    DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    formulation, kExactDerivativeEnforceSpd, nullptr, nullptr);
  auto assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation, std::move(elasticField), std::move(plasticField), nullptr);
  c.energy = std::make_unique<DeformationModelEnergy>(std::move(assembler), 0, false);
  c.numDOFs = c.energy->getNumDOFs();
  setVolumetricPlasticIdentity(*c.energy);

  checkGradientVsFDFunc(c, 1e-6, "MOONEY_RIVLIN");
  checkHessianVsFDGradient(c, 1e-5, "MOONEY_RIVLIN");
}

TEST(CubicTricubicHermiteMaterialSweepFDGTest, HillStableNeo) { runHermiteSweep(DeformationModelElasticMaterial::HILL_STABLE_NEO, true, "HILL_STABLE_NEO"); }
TEST(CubicTricubicHermiteMaterialSweepFDGTest, HillStVK) { runHermiteSweep(DeformationModelElasticMaterial::HILL_STVK, true, "HILL_STVK"); }
TEST(CubicTricubicHermiteMaterialSweepFDGTest, HillStVKVol) { runHermiteSweep(DeformationModelElasticMaterial::HILL_STVK_VOL, true, "HILL_STVK_VOL"); }
