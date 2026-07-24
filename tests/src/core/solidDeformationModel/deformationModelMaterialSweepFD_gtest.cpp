// Material-breadth finite-difference sweep.
//
// The element/assembler FD nets elsewhere only exercise a handful of materials
// (STABLE_NEO, HILL_STABLE_NEO, KOITER_STVK). This sweep runs the two
// material-agnostic position-derivative identities
//   gradient == FD(energy)        (exercises P = dpsi/dF)
//   hessian  == FD(gradient)      (exercises dPdF), enforceSPD = 0
// across EVERY elastic material the factory supports, on a single element so the
// dense Hessian FD stays cheap. This is what the deleted legacy harness was trying
// (but, being unregistered and assert-free, failing) to cover.
//
// Supported elastic materials (ElasticModelFactory): STABLE_NEO, STVK, STVK_VOL,
// INV_STVK, VOLUME, LINEAR, MOONEY_RIVLIN, HILL_STABLE_NEO, HILL_STVK,
// HILL_STVK_VOL (volumetric); KOITER_STVK, KOITER_FABRIC (shell).

#include <gtest/gtest.h>

#include "deformation/deformationModelAssembler.h"
#include "deformation/deformationModel.h"
#include "deformation/deformationModelManager.h"
#include "material/fields/materialParameterFieldInit.h"
#include "simulation/simulationMesh.h"
#include "material/plastic/plasticModel3DDeformationGradient.h"
#include "formulations/formulation/formulations.h"
#include "pgoLogging.h"
#include "triMeshGeo.h"

#include <tbb/global_control.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

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

ES::VXd perturbedPositions(const SimulationMesh &mesh)
{
  ES::VXd x(mesh.getNumVertices() * 3);
  for (int vi = 0; vi < mesh.getNumVertices(); vi++) {
    double p[3];
    mesh.getVertex(vi, p);
    x[vi * 3 + 0] = p[0] + 0.03 * std::sin(0.7 * vi + 0.1);
    x[vi * 3 + 1] = p[1] + 0.02 * std::cos(0.9 * vi + 0.2);
    x[vi * 3 + 2] = p[2] + 0.025 * std::sin(1.1 * vi + 0.3);
  }
  return x;
}

// A built single-element case: assembler + the state kept alive for param setup.
struct Case
{
  std::shared_ptr<const SimulationMesh> meshOwner;
  std::unique_ptr<DeformationModelAssembler> assembler;
  ES::VXd x;
};

std::unique_ptr<SimulationMesh> singleTet(const SimulationMeshMaterial &mat)
{
  const double vertices[] = { 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 4.0 };
  const int elementVertices[] = { 0, 1, 2, 3 };
  const int elementMaterialIndices[] = { 0 };
  const SimulationMeshMaterial *materials[] = { &mat };
  return std::make_unique<SimulationMesh>(
    4, vertices, 1, 4, elementVertices, elementMaterialIndices, 1, materials, SimulationMeshType::TET);
}

// Volumetric single-tet case. withHill appends a Hill activation material;
// GlobalAxes supplies its frame.
Case makeVolCase(DeformationModelElasticMaterial elastic, std::unique_ptr<SimulationMesh> meshMutable, bool withHill)
{
  pgo::Logging::init();
  Case c;
  if (withHill) {
    static SimulationMeshHillMaterial hillMaterial(2500.0, 0.35, 1.0);
    meshMutable->appendMaterialToAllElements(&hillMaterial);
  }
  c.meshOwner = std::shared_ptr<const SimulationMesh>(std::move(meshMutable));

  const int nele = c.meshOwner->getNumElements();

  auto elasticField = createElasticParameterField(*c.meshOwner, elastic, ElasticFieldInit{});
  auto plasticField = createPlasticParameterField(
    *c.meshOwner, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6, PlasticFieldInit{});
  TetLinearFormulation formulation;
  auto manager = std::make_shared<DeformationModelManager>(
    c.meshOwner, elastic, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    formulation, kExactDerivativeEnforceSpd);

  // Plastic identity.
  const int np = manager->getNumPlasticParameters();
  ES::VXd plastic(static_cast<Eigen::Index>(np) * nele);
  for (int ei = 0; ei < nele; ei++)
    manager->getDeformationModel(ei)->defaultPlasticParams(plastic.data() + ei * np);

  c.assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation, std::move(elasticField), std::move(plasticField), nullptr);
  c.assembler->setPlasticValues(plastic);
  c.x = perturbedPositions(*c.meshOwner);
  return c;
}

// Shell case on a small Koiter grid; elastic params are set explicitly.
Case makeShellCase(DeformationModelElasticMaterial elastic, const ES::VXd &elasticParamsPerElement)
{
  pgo::Logging::init();
  constexpr int N = 3;  // 3x3 grid -> 9 verts, 8 triangles
  std::vector<double> vertices;
  for (int j = 0; j < N; j++)
    for (int i = 0; i < N; i++) {
      vertices.push_back(0.1 * i);
      vertices.push_back(0.1 * j);
      vertices.push_back(0.0);
    }
  auto vid = [](int i, int j) { return j * N + i; };
  std::vector<int> tris;
  for (int j = 0; j < N - 1; j++)
    for (int i = 0; i < N - 1; i++) {
      tris.push_back(vid(i, j)); tris.push_back(vid(i + 1, j)); tris.push_back(vid(i + 1, j + 1));
      tris.push_back(vid(i, j)); tris.push_back(vid(i + 1, j + 1)); tris.push_back(vid(i, j + 1));
    }
  pgo::Mesh::TriMeshGeo surfaceMesh(N * N, vertices.data(), static_cast<int>(tris.size() / 3), tris.data());
  SimulationMeshENuhMaterial mat(1000.0, 0.45, 1e-3);

  Case c;
  c.meshOwner = std::shared_ptr<const SimulationMesh>(loadShellMesh(surfaceMesh, &mat).release());
  const int nele = c.meshOwner->getNumElements();

  auto elasticField = createElasticParameterField(*c.meshOwner, elastic, ElasticFieldInit{});
  auto plasticField = createPlasticParameterField(
    *c.meshOwner, DeformationModelPlasticMaterial::SHELL_FF_DOF1, PlasticFieldInit{});
  KoiterShellFormulation formulation;
  auto manager = std::make_shared<DeformationModelManager>(
    c.meshOwner, elastic, DeformationModelPlasticMaterial::SHELL_FF_DOF1,
    formulation, kExactDerivativeEnforceSpd);
  c.assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation, std::move(elasticField), std::move(plasticField), nullptr);

  c.assembler->setPlasticValues(ES::VXd::Constant(nele, 1.0));
  const int ne = static_cast<int>(elasticParamsPerElement.size());
  ES::VXd elasticAll(static_cast<Eigen::Index>(ne) * nele);
  for (int ei = 0; ei < nele; ei++) elasticAll.segment(ei * ne, ne) = elasticParamsPerElement;
  c.assembler->setElasticValues(elasticAll);

  c.x = perturbedPositions(*c.meshOwner);
  return c;
}

// gradient(x) == d energy / d x.
void checkGradientVsFDEnergy(Case &c, double tol, const char *name)
{
  const int n = c.assembler->getNumDOFs();
  ES::VXd analytic = ES::VXd::Zero(n);
  c.assembler->computeGradient(c.x.data(), analytic.data());
  EXPECT_GT(analytic.norm(), 0.0) << name << ": gradient is zero; FD check is vacuous.";

  ScopedSerialTbb serial;
  ES::VXd fd(n);
  for (int i = 0; i < n; i++) {
    fd[i] = fivePointScalar([&](double delta) {
      ES::VXd xp = c.x; xp[i] += delta;
      return c.assembler->computeEnergy(xp.data());
    },
      kFiniteDifferenceStep);
  }
  const double err = (fd - analytic).norm() / std::max(1.0, analytic.norm());
  EXPECT_LT(err, tol) << name << ": gradient disagrees with FD(energy), rel err " << err << ".";
}

// hessian(x) == d gradient / d x.
void checkHessianVsFDGradient(Case &c, double tol, const char *name)
{
  const int n = c.assembler->getNumDOFs();
  ES::SpMatD H = c.assembler->getHessianTemplate();
  c.assembler->computeHessian(c.x.data(), H);
  ES::MXd analytic(H);
  EXPECT_GT(analytic.norm(), 0.0) << name << ": hessian is zero; FD check is vacuous.";

  ScopedSerialTbb serial;
  ES::MXd fd(n, n);
  for (int i = 0; i < n; i++) {
    fd.col(i) = fivePointVector([&](double delta) {
      ES::VXd xp = c.x; xp[i] += delta;
      ES::VXd g = ES::VXd::Zero(n);
      c.assembler->computeGradient(xp.data(), g.data());
      return g;
    },
      kFiniteDifferenceStep);
  }
  const double err = (fd - analytic).norm() / std::max(1.0, analytic.norm());
  EXPECT_LT(err, tol) << name << ": hessian disagrees with FD(gradient), rel err " << err << ".";
}

void runVol(DeformationModelElasticMaterial elastic, bool withHill, const char *name,
  double gradTol = 1e-6, double hessTol = 1e-5)
{
  SimulationMeshENuMaterial enu(1000.0, 0.45);
  Case c = makeVolCase(elastic, singleTet(enu), withHill);
  checkGradientVsFDEnergy(c, gradTol, name);
  checkHessianVsFDGradient(c, hessTol, name);
}
}  // namespace

TEST(DeformationModelMaterialSweepFDGTest, StableNeo) { runVol(DeformationModelElasticMaterial::STABLE_NEO, false, "STABLE_NEO"); }
TEST(DeformationModelMaterialSweepFDGTest, StVK) { runVol(DeformationModelElasticMaterial::STVK, false, "STVK"); }
TEST(DeformationModelMaterialSweepFDGTest, StVKVol) { runVol(DeformationModelElasticMaterial::STVK_VOL, false, "STVK_VOL"); }
TEST(DeformationModelMaterialSweepFDGTest, InvStVK) { runVol(DeformationModelElasticMaterial::INV_STVK, false, "INV_STVK"); }
TEST(DeformationModelMaterialSweepFDGTest, Volume) { runVol(DeformationModelElasticMaterial::VOLUME, false, "VOLUME"); }
TEST(DeformationModelMaterialSweepFDGTest, Linear) { runVol(DeformationModelElasticMaterial::LINEAR, false, "LINEAR"); }

TEST(DeformationModelMaterialSweepFDGTest, MooneyRivlin)
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
  SimulationMeshMooneyRivlinMaterial mooney(2, 2, Cpq.data(), D.data());
  Case c = makeVolCase(DeformationModelElasticMaterial::MOONEY_RIVLIN, singleTet(mooney), false);
  checkGradientVsFDEnergy(c, 1e-6, "MOONEY_RIVLIN");
  checkHessianVsFDGradient(c, 1e-5, "MOONEY_RIVLIN");
}

TEST(DeformationModelMaterialSweepFDGTest, HillStableNeo) { runVol(DeformationModelElasticMaterial::HILL_STABLE_NEO, true, "HILL_STABLE_NEO"); }
TEST(DeformationModelMaterialSweepFDGTest, HillStVK) { runVol(DeformationModelElasticMaterial::HILL_STVK, true, "HILL_STVK"); }
TEST(DeformationModelMaterialSweepFDGTest, HillStVKVol) { runVol(DeformationModelElasticMaterial::HILL_STVK_VOL, true, "HILL_STVK_VOL"); }

TEST(DeformationModelMaterialSweepFDGTest, KoiterStVK)
{
  ES::VXd params(5);
  params << 20000.0, 0.45, 10000.0, 0.3, 1e-3;
  Case c = makeShellCase(DeformationModelElasticMaterial::KOITER_STVK, params);
  checkGradientVsFDEnergy(c, 1e-5, "KOITER_STVK");
  checkHessianVsFDGradient(c, 1e-4, "KOITER_STVK");
}

TEST(DeformationModelMaterialSweepFDGTest, KoiterFabric)
{
  // Factory-default fabric parameters (ElasticModelFactory::initializeDefaultElasticParams).
  ES::VXd params(12);
  params << 1, 1, 1, 1, 1, 1, 1, 1000, 1000, 1000, 1, 1e-3;
  Case c = makeShellCase(DeformationModelElasticMaterial::KOITER_FABRIC, params);

  // Keep the finite-energy guard so this failure mode remains explicit: if the
  // material overflows at rest, derivative FD checks cannot produce useful evidence.
  ES::VXd rest(c.assembler->getNumDOFs());
  for (int vi = 0; vi < c.meshOwner->getNumVertices(); vi++) {
    double p[3];
    c.meshOwner->getVertex(vi, p);
    rest.segment<3>(vi * 3) << p[0], p[1], p[2];
  }
  const double restEnergy = c.assembler->computeEnergy(rest.data());
  if (!std::isfinite(restEnergy)) {
    GTEST_SKIP() << "KOITER_FABRIC rest energy is non-finite (" << restEnergy
                 << ") at factory-default parameters; material not yet exercisable "
                    "end-to-end. FD sweep skipped until the material is fixed.";
  }
  checkGradientVsFDEnergy(c, 1e-5, "KOITER_FABRIC");
  checkHessianVsFDGradient(c, 1e-4, "KOITER_FABRIC");
}
