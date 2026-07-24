// Full-chain finite-difference tests for DeformationModelEnergy.
//
// DeformationModelEnergy is the displacement-state wrapper around the assembler:
// func/gradient/hessian all receive a displacement u, add the rest position, apply
// the energy DOF offset, and scatter into the assembler. Differentiating through
// that whole chain (rather than the assembler in isolation) is what catches state
// convention, rest-position, offset and scatter bugs.
//
// Two derivative identities are checked on small hand-built meshes (tet, hex,
// Koiter shell) so the dense Hessian finite difference stays cheap:
//   * gradient(u) == d func / d u
//   * hessian(u)  == d gradient / d u   (built with enforceSPD = 0 so the assembled
//                                        Hessian is the true derivative, not the
//                                        SPD-projected optimizer Hessian)

#include <gtest/gtest.h>

#include "deformation/deformationModelAssembler.h"
#include "energy/deformationModelEnergy.h"
#include "deformation/deformationModelManager.h"
#include "formulations/formulation/formulations.h"
#include "material/fields/materialParameterFieldInit.h"
#include "material/plastic/plasticModel3DDeformationGradient.h"
#include "simulation/simulationMesh.h"
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
using pgo::SolidDeformationModel::DeformationModelAssembler;
using pgo::SolidDeformationModel::DeformationModelElasticMaterial;
using pgo::SolidDeformationModel::DeformationModelEnergy;
using pgo::SolidDeformationModel::DeformationModelManager;
using pgo::SolidDeformationModel::DeformationModelPlasticMaterial;
using pgo::SolidDeformationModel::ElasticFieldInit;
using pgo::SolidDeformationModel::PlasticFieldInit;
using pgo::SolidDeformationModel::createElasticParameterField;
using pgo::SolidDeformationModel::createPlasticParameterField;
using pgo::SolidDeformationModel::PlasticModel3DDeformationGradient;
using pgo::SolidDeformationModel::SimulationMesh;
using pgo::SolidDeformationModel::SimulationMeshENuhMaterial;
using pgo::SolidDeformationModel::SimulationMeshENuMaterial;
using pgo::SolidDeformationModel::SimulationMeshType;

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
  ES::VXd gp2 = eval(2.0 * h);
  ES::VXd gp1 = eval(h);
  ES::VXd gm1 = eval(-h);
  ES::VXd gm2 = eval(-2.0 * h);
  return (-gp2 + 8.0 * gp1 - 8.0 * gm1 + gm2) / (12.0 * h);
}

// A built energy plus the supporting objects that must outlive it.
struct EnergyCase
{
  std::shared_ptr<const SimulationMesh> meshOwner;
  std::unique_ptr<DeformationModelEnergy> energy;
  int offset = 0;
  int numDOFs = 0;
};

template<class FormulationT>
std::unique_ptr<DeformationModelEnergy> finalizeEnergy(
  std::shared_ptr<const SimulationMesh> mesh,
  DeformationModelElasticMaterial elastic,
  DeformationModelPlasticMaterial plastic,
  const FormulationT &formulation,
  int offset)
{
  auto elasticField = createElasticParameterField(*mesh, elastic, ElasticFieldInit{});
  auto plasticField = createPlasticParameterField(*mesh, plastic, PlasticFieldInit{});
  auto manager = std::make_shared<DeformationModelManager>(
    mesh, elastic, plastic, formulation, kExactDerivativeEnforceSpd, nullptr, nullptr);
  auto assembler = std::make_unique<DeformationModelAssembler>(
    std::move(manager), formulation, std::move(elasticField), std::move(plasticField), nullptr);
  // enableMaterialMaxStep = false: the max-step clamp is irrelevant to derivative FD
  // and only adds logging noise.
  return std::make_unique<DeformationModelEnergy>(std::move(assembler), offset, false);
}

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

// A smooth, element-valid local displacement so the gradient/Hessian are
// nontrivially nonzero.
ES::VXd makeSmoothDisplacement(int nvtx)
{
  ES::VXd u = ES::VXd::Zero(static_cast<Eigen::Index>(nvtx) * 3);
  for (int vi = 0; vi < nvtx; vi++) {
    u[vi * 3 + 0] = 5e-3 * std::sin(0.9 * vi + 0.1);
    u[vi * 3 + 1] = 4e-3 * std::cos(0.7 * vi + 0.3);
    u[vi * 3 + 2] = 3e-3 * std::sin(1.3 * vi + 0.5);
  }
  return u;
}

EnergyCase makeSingleTetCase(int offset)
{
  pgo::Logging::init();
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3 };
  const int elementMaterialIndices[] = { 0 };
  SimulationMeshENuMaterial baseMaterial(1200.0, 0.45);
  const pgo::SolidDeformationModel::SimulationMeshMaterial *materials[] = { &baseMaterial };

  EnergyCase c;
  c.meshOwner = std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    4, vertices, 1, 4, elementVertices, elementMaterialIndices, 1, materials, SimulationMeshType::TET));
  c.energy = finalizeEnergy(
    c.meshOwner, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    pgo::SolidDeformationModel::TetLinearFormulation{}, offset);
  c.offset = offset;
  c.numDOFs = c.energy->getNumDOFs();
  setVolumetricPlasticIdentity(*c.energy);
  return c;
}

EnergyCase makeSingleHexCase(int offset)
{
  pgo::Logging::init();
  const double vertices[] = {
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  const int elementMaterialIndices[] = { 0 };
  SimulationMeshENuMaterial baseMaterial(1200.0, 0.45);
  const pgo::SolidDeformationModel::SimulationMeshMaterial *materials[] = { &baseMaterial };

  EnergyCase c;
  c.meshOwner = std::shared_ptr<const SimulationMesh>(new SimulationMesh(
    8, vertices, 1, 8, elementVertices, elementMaterialIndices, 1, materials, SimulationMeshType::CUBIC));
  c.energy = finalizeEnergy(
    c.meshOwner, DeformationModelElasticMaterial::STABLE_NEO, DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
    pgo::SolidDeformationModel::CubicLinearFormulation{}, offset);
  c.offset = offset;
  c.numDOFs = c.energy->getNumDOFs();
  setVolumetricPlasticIdentity(*c.energy);
  return c;
}

EnergyCase makeShellPatchCase(int offset)
{
  pgo::Logging::init();
  // 4x4 flat grid in the z = 0 plane, two triangles per cell.
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

  EnergyCase c;
  c.meshOwner = std::shared_ptr<const SimulationMesh>(
    pgo::SolidDeformationModel::loadShellMesh(surfaceMesh, &mat).release());
  c.energy = finalizeEnergy(
    c.meshOwner, DeformationModelElasticMaterial::KOITER_STVK, DeformationModelPlasticMaterial::SHELL_FF_DOF1,
    pgo::SolidDeformationModel::KoiterShellFormulation{}, offset);
  c.offset = offset;
  c.numDOFs = c.energy->getNumDOFs();

  const int nele = c.meshOwner->getNumElements();
  // Plastic stretch identity (1.0) and a reasonable elastic parameter set.
  ES::VXd plastic = ES::VXd::Constant(nele, 1.0);
  c.energy->assembler().setPlasticValues(plastic);
  ES::VXd elastic(static_cast<Eigen::Index>(nele) * 5);
  for (int ei = 0; ei < nele; ei++)
    elastic.segment<5>(ei * 5) << 20000.0, 0.45, 10000.0, 0.3, 1e-3;
  c.energy->assembler().setElasticValues(elastic);
  return c;
}

// gradient(u) == d func / d u for the energy's local displacement state.
void checkGradientVsFDFunc(EnergyCase &c, double tol)
{
  const int n = c.numDOFs;
  ES::VXd u = makeSmoothDisplacement(c.meshOwner->getNumVertices());

  ES::VXd analytic(n);
  c.energy->gradient(u, analytic);

  ScopedSerialTbb serial;
  ES::VXd fd(n);
  for (int i = 0; i < n; i++) {
    fd[i] = fivePointScalar([&](double delta) {
      ES::VXd up = u;
      up[i] += delta;
      return c.energy->func(up);
    },
      kFiniteDifferenceStep);
  }

  const double err = (fd - analytic).norm() / std::max(1.0, analytic.norm());
  EXPECT_LT(err, tol) << "Energy gradient disagrees with FD of func (rel err " << err << ").";
  EXPECT_GT(analytic.norm(), 0.0) << "Gradient is identically zero; the FD check is vacuous.";
}

// hessian(u) == d gradient / d u. Requires enforceSPD = 0 (built that way above).
void checkHessianVsFDGradient(EnergyCase &c, double tol)
{
  const int n = c.numDOFs;
  ES::VXd u = makeSmoothDisplacement(c.meshOwner->getNumVertices());

  ES::SpMatD H;
  c.energy->hessianAlloc(H);
  c.energy->hessianInPlace(u, H);
  ES::MXd analytic(H);
  ASSERT_EQ(analytic.rows(), n);
  ASSERT_EQ(analytic.cols(), n);

  ScopedSerialTbb serial;
  ES::MXd fd(n, n);
  for (int i = 0; i < n; i++) {
    ES::VXd col = fivePointVector([&](double delta) {
      ES::VXd up = u;
      up[i] += delta;
      ES::VXd g(n);
      c.energy->gradient(up, g);
      return g;
    },
      kFiniteDifferenceStep);
    fd.col(i) = col;
  }

  const double err = (fd - analytic).norm() / std::max(1.0, analytic.norm());
  EXPECT_LT(err, tol) << "Energy Hessian disagrees with FD of gradient (rel err " << err << ").";
  EXPECT_GT(analytic.norm(), 0.0) << "Hessian is identically zero; the FD check is vacuous.";
}
}  // namespace

TEST(DeformationModelEnergyFDGTest, TetGradientMatchesFiniteDifferenceOfFunc)
{
  auto c = makeSingleTetCase(0);
  checkGradientVsFDFunc(c, 1e-6);
}

TEST(DeformationModelEnergyFDGTest, TetGradientMatchesFiniteDifferenceWithOffset)
{
  // The offset controls global assembly DOFs; direct energy evaluation remains local.
  auto c = makeSingleTetCase(6);
  std::vector<int> dofs;
  c.energy->getDOFs(dofs);
  ASSERT_EQ(dofs.size(), static_cast<std::size_t>(c.numDOFs));
  EXPECT_EQ(dofs.front(), 6);
  EXPECT_EQ(dofs.back(), 6 + c.numDOFs - 1);
  checkGradientVsFDFunc(c, 1e-6);
}

TEST(DeformationModelEnergyFDGTest, TetHessianMatchesFiniteDifferenceOfGradient)
{
  auto c = makeSingleTetCase(0);
  checkHessianVsFDGradient(c, 1e-6);
}

TEST(DeformationModelEnergyFDGTest, HexGradientMatchesFiniteDifferenceOfFunc)
{
  auto c = makeSingleHexCase(0);
  checkGradientVsFDFunc(c, 1e-6);
}

TEST(DeformationModelEnergyFDGTest, HexHessianMatchesFiniteDifferenceOfGradient)
{
  auto c = makeSingleHexCase(0);
  checkHessianVsFDGradient(c, 1e-6);
}

TEST(DeformationModelEnergyFDGTest, ShellGradientMatchesFiniteDifferenceOfFunc)
{
  auto c = makeShellPatchCase(0);
  checkGradientVsFDFunc(c, 1e-5);
}

TEST(DeformationModelEnergyFDGTest, ShellHessianMatchesFiniteDifferenceOfGradient)
{
  // Built with enforceSPD = 0, so this also pins the *unclamped* shell Hessian.
  auto c = makeShellPatchCase(0);
  checkHessianVsFDGradient(c, 1e-5);
}
