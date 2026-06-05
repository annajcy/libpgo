#include "gtest/gtest.h"

#include "formulations/formulation.h"
#include "formulations/formulationDynamics.h"
#include "barycentricCoordinates.h"
#include "cubicMesh.h"
#include "generateMassMatrix.h"

#include <memory>
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

}  // namespace

TEST(FormulationDynamicsGTest, HermiteMassHasCorrectShapeSymmetryAndConstantVelocityEnergy)
{
  auto mesh = makeSingleCube(2.0);
  EigenSupport::SpMatD M = buildFormulationMassMatrix(*mesh, TricubicHermiteFormulation{});
  ASSERT_EQ(M.rows(), 8 * 24);
  ASSERT_EQ(M.cols(), 8 * 24);

  EigenSupport::VXd qdot = EigenSupport::VXd::Zero(8 * 24);
  EigenSupport::V3d v(0.4, -0.2, 0.7);
  for (int vertex = 0; vertex < 8; vertex++)
    qdot.segment<3>(vertex * 24) = v;

  const double kinetic = 0.5 * qdot.dot(M * qdot);
  const double expected = 0.5 * 2.0 * v.squaredNorm();
  EXPECT_NEAR(kinetic, expected, 1e-10);

  for (int k = 0; k < M.outerSize(); k++) {
    for (EigenSupport::SpMatD::InnerIterator it(M, k); it; ++it) {
      EXPECT_NEAR(it.value(), sparseCoeff(M, it.col(), it.row()), 1e-12);
    }
  }
}

TEST(FormulationDynamicsGTest, HermiteBodyForceHasCorrectTotalAndDerivativeEntries)
{
  auto mesh = makeSingleCube(3.0);
  EigenSupport::V3d a(0.0, -9.8, 0.0);
  EigenSupport::VXd f = buildFormulationBodyForce(*mesh, TricubicHermiteFormulation{}, a);
  ASSERT_EQ(f.size(), 8 * 24);

  EigenSupport::V3d valueForce = EigenSupport::V3d::Zero();
  double derivativeNorm = 0.0;
  for (int vertex = 0; vertex < 8; vertex++) {
    valueForce += f.segment<3>(vertex * 24);
    derivativeNorm += f.segment(vertex * 24 + 3, 21).norm();
  }

  EXPECT_TRUE(valueForce.isApprox(3.0 * a, 1e-10));
  EXPECT_GT(derivativeNorm, 0.0);
}

TEST(FormulationDynamicsGTest, HermiteSurfaceEmbeddingReproducesAffineDisplacement)
{
  auto mesh = makeSingleCube(1.0);
  EigenSupport::MXd points(3, 3);
  points << 0.25, 0.50, 0.75,
            1.00, 0.00, 0.50,
            0.00, 1.00, 0.00;

  EigenSupport::SpMatD W = buildFormulationSurfaceEmbeddingMatrix(*mesh, TricubicHermiteFormulation{}, points);
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
  auto mesh = makeSingleCube(2.0);
  EigenSupport::SpMatD legacyMass;
  VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(mesh.get(), legacyMass, true);

  EigenSupport::SpMatD mass = buildFormulationMassMatrix(*mesh, LinearCubicFormulation{});
  EXPECT_TRUE(mass.isApprox(legacyMass, 1e-12));
}

TEST(FormulationDynamicsGTest, TrilinearSurfaceEmbeddingMatchesLegacyBarycentricOperator)
{
  auto mesh = makeSingleCube(1.0);
  EigenSupport::MXd points(2, 3);
  points << 0.25, 0.50, 0.75,
            1.00, 0.00, 0.50;

  EigenSupport::SpMatD W = buildFormulationSurfaceEmbeddingMatrix(*mesh, LinearCubicFormulation{}, points);

  std::vector<double> flat = {
    0.25, 0.50, 0.75,
    1.00, 0.00, 0.50,
  };
  InterpolationCoordinates::BarycentricCoordinates bc(2, flat.data(), mesh.get());
  EigenSupport::SpMatD legacyW = bc.generateInterpolationMatrix();

  EXPECT_TRUE(W.isApprox(legacyW, 1e-12));
}

TEST(FormulationDynamicsGTest, HermiteBoundaryHelpersExposePolicies)
{
  std::vector<int> value = hermiteVertexDofs({ 2 }, HermiteBoundaryPolicy::Value);
  std::vector<int> first = hermiteVertexDofs({ 2 }, HermiteBoundaryPolicy::First);
  std::vector<int> all = hermiteVertexDofs({ 2 }, HermiteBoundaryPolicy::All);

  ASSERT_EQ(value.size(), 3);
  EXPECT_EQ(value.front(), 48);
  ASSERT_EQ(first.size(), 12);
  EXPECT_EQ(first.front(), 48);
  EXPECT_EQ(first.back(), 59);
  ASSERT_EQ(all.size(), 24);
  EXPECT_EQ(all.front(), 48);
  EXPECT_EQ(all.back(), 71);
}
