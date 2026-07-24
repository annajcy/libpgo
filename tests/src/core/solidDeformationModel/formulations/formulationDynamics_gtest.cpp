#include "gtest/gtest.h"

#include "formulations/formulation/formulations.h"
#include "barycentricCoordinates.h"
#include "cubicMesh.h"
#include "generateMassMatrix.h"
#include "mass/shellDensityElasticThickness.h"
#include "mass/volumeMassField.h"
#include "material/fields/materialParameters.h"
#include "simulation/simulationMesh.h"
#include "triMeshGeo.h"

#include <cmath>
#include <memory>
#include <span>
#include <vector>

using namespace pgo;
using namespace pgo::SolidDeformationModel;

namespace
{

std::unique_ptr<VolumetricMeshes::CubicMesh> makeSingleCube(double density)
{
  double vertices[24] = {
    0, 0, 0,
    1, 0, 0,
    1, 1, 0,
    0, 1, 0,
    0, 0, 1,
    1, 0, 1,
    1, 1, 1,
    0, 1, 1,
  };
  int elements[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  return std::make_unique<VolumetricMeshes::CubicMesh>(
    8, vertices, 1, elements, 1e6, 0.45, density);
}

double sparseCoeff(const EigenSupport::SpMatD &M, int r, int c)
{
  return M.coeff(r, c);
}

class SquareParameterMapping final : public ParameterFieldMapping
{
public:
  explicit SquareParameterMapping(int size): size_(size) {}

  int numInputDofs() const override { return size_; }
  int numChannels() const override { return size_; }
  bool isAffine() const override { return false; }

  void evaluate(
    int, int, std::span<const double> z,
    std::span<double> p) const override
  {
    for (int i = 0; i < size_; i++)
      p[i] = z[i] * z[i];
  }

  void evaluateJacobian(
    int, int, std::span<const double> z,
    double *output) const override
  {
    std::fill(output, output + size_ * size_, 0.0);
    for (int i = 0; i < size_; i++)
      output[i * size_ + i] = 2.0 * z[i];
  }

  void evaluateHessians(
    int, int, std::span<const double>,
    double *output) const override
  {
    std::fill(output, output + size_ * size_ * size_, 0.0);
    for (int i = 0; i < size_; i++)
      output[i * size_ * size_ + i * size_ + i] = 2.0;
  }

private:
  int size_;
};

std::shared_ptr<const SimulationMesh> makeTwoTriangleShellMesh()
{
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    1.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
  };
  const int triangles[] = {
    0, 1, 2,
    0, 2, 3,
  };
  Mesh::TriMeshGeo surface(4, vertices, 2, triangles);
  SimulationMeshENuhMaterial material(1000.0, 0.35, 1e-3);
  return std::shared_ptr<const SimulationMesh>(
    loadShellMesh(surface, &material).release());
}

std::shared_ptr<MaterialParameters> makeShellMassParameters(
  int numElements, bool constant, bool nonlinear)
{
  constexpr int numElasticChannels = 5;
  std::unique_ptr<const ParameterDofLayout> elasticLayout;
  if (constant) {
    elasticLayout = std::make_unique<ConstantParameterDofLayout>(
      numElements, numElasticChannels);
  }
  else {
    elasticLayout = std::make_unique<ElementwiseParameterDofLayout>(
      numElements, numElasticChannels);
  }

  std::unique_ptr<const ParameterFieldMapping> elasticMapping;
  if (nonlinear)
    elasticMapping = std::make_unique<SquareParameterMapping>(numElasticChannels);
  else
    elasticMapping = std::make_unique<IdentityParameterFieldMapping>(numElasticChannels);

  MaterialParameterBlock elasticBlock(
    MaterialParameterBlockKind::ELASTIC, "koiter_stvk",
    { "E_membrane", "nu_membrane", "E_bending", "nu_bending", "thickness" },
    std::move(elasticLayout), std::move(elasticMapping));
  MaterialParameterBlock plasticBlock(
    MaterialParameterBlockKind::PLASTIC, "none", {},
    std::make_unique<ElementwiseParameterDofLayout>(numElements, 0),
    std::make_unique<IdentityParameterFieldMapping>(0));
  auto space = std::make_shared<MaterialParameterSpace>(
    std::move(elasticBlock), std::move(plasticBlock));

  const int rows = constant ? 1 : numElements;
  EigenSupport::VXd elastic(rows * numElasticChannels);
  for (int row = 0; row < rows; row++) {
    elastic.segment<5>(row * numElasticChannels) <<
      2.0, 0.4, 1.5, 0.3, 0.025 + 0.004 * row;
  }
  return std::make_shared<MaterialParameters>(
    std::move(space), std::move(elastic), EigenSupport::VXd());
}

void expectBodyForceParameterJacobianMatchesFD(
  bool constant, bool nonlinear)
{
  auto mesh = makeTwoTriangleShellMesh();
  auto parameters = makeShellMassParameters(
    mesh->getNumElements(), constant, nonlinear);
  auto space = parameters->space();
  ShellDensityElasticThickness massField(
    850.0, space, space->elastic().parameter("thickness"));
  KoiterShellFormulation formulation;
  const EigenSupport::V3d acceleration(0.7, -1.3, -9.81);
  const EigenSupport::VXd z = parameters->elasticSnapshot();

  const EigenSupport::SpMatD jacobian =
    formulation.buildBodyForceParameterJacobian(
      *mesh, acceleration, massField, parameters->committedView());
  EigenSupport::MXd fd(jacobian.rows(), jacobian.cols());
  constexpr double h = 1e-7;
  for (int col = 0; col < z.size(); col++) {
    EigenSupport::VXd zp = z;
    EigenSupport::VXd zm = z;
    zp[col] += h;
    zm[col] -= h;
    auto vp = space->makeStateView(
      std::span<const double>(zp.data(), zp.size()), {});
    auto vm = space->makeStateView(
      std::span<const double>(zm.data(), zm.size()), {});
    fd.col(col) = (
      formulation.buildBodyForce(*mesh, acceleration, massField, vp) -
      formulation.buildBodyForce(*mesh, acceleration, massField, vm)) /
      (2.0 * h);
  }

  EXPECT_LT(
    (EigenSupport::MXd(jacobian) - fd).norm() /
      std::max(1.0, EigenSupport::MXd(jacobian).norm()),
    2e-8);
}

}  // namespace

TEST(FormulationDynamicsGTest, HermiteMassHasCorrectShapeSymmetryAndConstantVelocityEnergy)
{
  constexpr double density = 2.0;
  auto mesh = makeSingleCube(density);
  auto simMesh = loadCubicMesh(mesh.get());
  ConstantVolumeDensity massField(density);
  EigenSupport::SpMatD M = CubicTricubicHermiteFormulation{}.buildMassMatrix(*simMesh, massField);
  ASSERT_EQ(M.rows(), 8 * 24);
  ASSERT_EQ(M.cols(), 8 * 24);

  EigenSupport::VXd qdot = EigenSupport::VXd::Zero(8 * 24);
  EigenSupport::V3d v(0.4, -0.2, 0.7);
  for (int vertex = 0; vertex < 8; vertex++)
    qdot.segment<3>(vertex * 24) = v;

  const double kinetic = 0.5 * qdot.dot(M * qdot);
  const double expected = 0.5 * density * v.squaredNorm();
  EXPECT_NEAR(kinetic, expected, 1e-10);

  for (int k = 0; k < M.outerSize(); k++) {
    for (EigenSupport::SpMatD::InnerIterator it(M, k); it; ++it) {
      EXPECT_NEAR(it.value(), sparseCoeff(M, it.col(), it.row()), 1e-12);
    }
  }
}

TEST(FormulationDynamicsGTest, HermiteBodyForceHasCorrectTotalAndDerivativeEntries)
{
  constexpr double density = 3.0;
  auto mesh = makeSingleCube(density);
  auto simMesh = loadCubicMesh(mesh.get());
  ConstantVolumeDensity massField(density);
  EigenSupport::V3d a(0.0, -9.8, 0.0);
  EigenSupport::VXd f = CubicTricubicHermiteFormulation{}.buildBodyForce(*simMesh, a, massField);
  ASSERT_EQ(f.size(), 8 * 24);

  EigenSupport::V3d valueForce = EigenSupport::V3d::Zero();
  double derivativeNorm = 0.0;
  for (int vertex = 0; vertex < 8; vertex++) {
    valueForce += f.segment<3>(vertex * 24);
    derivativeNorm += f.segment(vertex * 24 + 3, 21).norm();
  }

  EXPECT_TRUE(valueForce.isApprox(density * a, 1e-10));
  EXPECT_GT(derivativeNorm, 0.0);
}

TEST(FormulationDynamicsGTest, HermiteSurfaceEmbeddingReproducesAffineDisplacement)
{
  auto mesh = makeSingleCube(1.0);
  EigenSupport::MXd points(3, 3);
  points << 0.25, 0.50, 0.75,
            1.00, 0.00, 0.50,
            0.00, 1.00, 0.00;

  EigenSupport::SpMatD W = CubicTricubicHermiteFormulation{}.buildSurfaceEmbeddingMatrix(*mesh, points);
  ASSERT_EQ(W.rows(), points.rows() * 3);
  ASSERT_EQ(W.cols(), 8 * 24);

  EigenSupport::VXd q = EigenSupport::VXd::Zero(8 * 24);
  EigenSupport::M3d A;
  A << 0.1, 0.2, 0.0,
       0.0, -0.1, 0.3,
       0.05, 0.0, 0.2;
  EigenSupport::V3d b(0.3, -0.4, 0.2);
  for (int vertex = 0; vertex < 8; vertex++) {
    const auto &X = mesh->getVertex(vertex);
    const int base = vertex * 24;
    q.segment<3>(base) = A * EigenSupport::V3d(X[0], X[1], X[2]) + b;
    q.segment<3>(base + 3) = A.col(0);
    q.segment<3>(base + 6) = A.col(1);
    q.segment<3>(base + 9) = A.col(2);
  }

  EigenSupport::VXd mapped = W * q;
  for (int i = 0; i < points.rows(); i++) {
    EigenSupport::V3d p = points.row(i).transpose();
    EXPECT_TRUE(mapped.segment<3>(i * 3).isApprox(A * p + b, 1e-12));
  }
}

TEST(FormulationDynamicsGTest, TrilinearMassMatchesLegacyOperator)
{
  constexpr double density = 2.0;
  auto mesh = makeSingleCube(density);
  auto simMesh = loadCubicMesh(mesh.get());
  ConstantVolumeDensity massField(density);
  EigenSupport::SpMatD legacyMass;
  VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(mesh.get(), legacyMass, true);

  EigenSupport::SpMatD mass = CubicLinearFormulation{}.buildMassMatrix(*simMesh, massField);
  EXPECT_TRUE(mass.isApprox(legacyMass, 1e-6));
}

TEST(FormulationDynamicsGTest, TrilinearSurfaceEmbeddingMatchesLegacyBarycentricOperator)
{
  auto mesh = makeSingleCube(1.0);
  EigenSupport::MXd points(2, 3);
  points << 0.25, 0.50, 0.75,
            1.00, 0.00, 0.50;

  EigenSupport::SpMatD W = CubicLinearFormulation{}.buildSurfaceEmbeddingMatrix(*mesh, points);

  std::vector<double> flat = {
    0.25, 0.50, 0.75,
    1.00, 0.00, 0.50,
  };
  InterpolationCoordinates::BarycentricCoordinates bc(2, flat.data(), mesh.get());
  EigenSupport::SpMatD legacyW = bc.generateInterpolationMatrix();

  EXPECT_TRUE(W.isApprox(legacyW, 1e-12));
}

TEST(FormulationDynamicsGTest, CubicSurfaceEmbeddingClampsPointsToElementBoundary)
{
  auto mesh = makeSingleCube(1.0);
  EigenSupport::MXd point(1, 3);
  point << -1.0, 2.0, 0.25;

  EigenSupport::VXd linearDofs(8 * 3);
  EigenSupport::VXd hermiteDofs = EigenSupport::VXd::Zero(8 * 24);
  for (int vertex = 0; vertex < 8; vertex++) {
    const auto &X = mesh->getVertex(vertex);
    const EigenSupport::V3d value(X[0] + 2.0 * X[1], 3.0 * X[2], X[0] - X[1]);
    linearDofs.segment<3>(vertex * 3) = value;
    hermiteDofs.segment<3>(vertex * 24) = value;
    hermiteDofs.segment<3>(vertex * 24 + 3) = EigenSupport::V3d(1.0, 0.0, 1.0);
    hermiteDofs.segment<3>(vertex * 24 + 6) = EigenSupport::V3d(2.0, 0.0, -1.0);
    hermiteDofs.segment<3>(vertex * 24 + 9) = EigenSupport::V3d(0.0, 3.0, 0.0);
  }

  const EigenSupport::V3d expected(2.0, 0.75, -1.0);  // field at clamped point (0, 1, 0.25)
  const EigenSupport::VXd linear = CubicLinearFormulation{}.buildSurfaceEmbeddingMatrix(*mesh, point) * linearDofs;
  const EigenSupport::VXd hermite = CubicTricubicHermiteFormulation{}.buildSurfaceEmbeddingMatrix(*mesh, point) * hermiteDofs;

  EXPECT_TRUE(linear.isApprox(expected, 1e-12)) << linear.transpose();
  EXPECT_TRUE(hermite.isApprox(expected, 1e-12)) << hermite.transpose();
}

TEST(FormulationDynamicsGTest, ConstantMappedThicknessBodyForceJacobianMatchesFD)
{
  expectBodyForceParameterJacobianMatchesFD(true, false);
}

TEST(FormulationDynamicsGTest, NonlinearElementwiseThicknessBodyForceJacobianMatchesFD)
{
  expectBodyForceParameterJacobianMatchesFD(false, true);
}
