#include <gtest/gtest.h>

#include "parallel/parallelControl.h"
#include "pgo_c.h"

#include <array>
#include <cmath>
#include <vector>

#ifndef PGO_TEST_MAX_CONCURRENCY
#  error "PGO_TEST_MAX_CONCURRENCY must be defined"
#endif

namespace
{

TEST(PgoCParallelMigrationTest, ClosestTriangleQueriesMatchAnalyticDistances)
{
  pgo::parallel::GlobalTbbControl control(PGO_TEST_MAX_CONCURRENCY);

  std::array<double, 9> vertices = {
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
  };
  std::array<int, 3> triangles = { 0, 1, 2 };
  pgoTriMeshGeoStructHandle mesh =
    pgo_create_trimeshgeo(3, vertices.data(), 1, triangles.data());
  ASSERT_NE(mesh, nullptr);

  constexpr int queryCount = 64;
  std::vector<double> queries(queryCount * 3);
  std::vector<double> expectedDistance2(queryCount);
  for (int i = 0; i < queryCount; ++i) {
    const double x = 0.05 + 0.005 * (i % 8);
    const double y = 0.10 + 0.005 * (i / 8);
    const double z = -0.4 + 0.0125 * i;
    queries[3 * i] = x;
    queries[3 * i + 1] = y;
    queries[3 * i + 2] = z;
    expectedDistance2[i] = z * z;
  }

  std::vector<double> distances(queryCount, -1.0);
  std::vector<int> triangleIDs(queryCount, -1);
  pgo_trimesh_closest_distances(
    mesh, queryCount, queries.data(), distances.data(), triangleIDs.data());

  for (int i = 0; i < queryCount; ++i) {
    EXPECT_EQ(triangleIDs[i], 0);
    EXPECT_NEAR(distances[i], expectedDistance2[i], 1e-12);
  }

  pgo_destroy_trimeshgeo(mesh);
}

TEST(PgoCParallelMigrationTest, TetBarycentricQueriesReconstructInput)
{
  pgo::parallel::GlobalTbbControl control(PGO_TEST_MAX_CONCURRENCY);

  std::array<double, 12> vertices = {
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
  };
  std::array<int, 4> tets = { 0, 1, 2, 3 };
  pgoTetMeshGeoStructHandle mesh = pgo_create_tetmeshgeo(4, vertices.data(), 1, tets.data());
  ASSERT_NE(mesh, nullptr);

  constexpr int queryCount = 64;
  std::vector<double> queries(queryCount * 3);
  for (int i = 0; i < queryCount; ++i) {
    queries[3 * i] = 0.10 + 0.002 * (i % 8);
    queries[3 * i + 1] = 0.15 + 0.002 * (i / 8);
    queries[3 * i + 2] = 0.20 + 0.001 * (i % 5);
  }

  std::vector<double> weights(queryCount * 4, 0.0);
  std::vector<int> elementIDs(queryCount, -1);
  pgo_tetmesh_barycentric_weights(
    mesh, queryCount, queries.data(), weights.data(), elementIDs.data());

  for (int i = 0; i < queryCount; ++i) {
    EXPECT_EQ(elementIDs[i], 0);
    double sum = 0.0;
    std::array<double, 3> reconstructed = { 0.0, 0.0, 0.0 };
    for (int vertex = 0; vertex < 4; ++vertex) {
      const double weight = weights[4 * i + vertex];
      sum += weight;
      for (int axis = 0; axis < 3; ++axis)
        reconstructed[axis] += weight * vertices[3 * vertex + axis];
    }
    EXPECT_NEAR(sum, 1.0, 1e-12);
    for (int axis = 0; axis < 3; ++axis)
      EXPECT_NEAR(reconstructed[axis], queries[3 * i + axis], 1e-12);
  }

  pgo_destroy_tetmeshgeo(mesh);
}

}  // namespace
