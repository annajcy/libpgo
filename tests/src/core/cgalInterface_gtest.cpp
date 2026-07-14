#include <gtest/gtest.h>

#include "cgalInterface.h"
#include "triMeshGeo.h"

#include <chrono>
#include <vector>

TEST(CGALInterfaceTest, DenseQrSmoothingAndRepairRemainAvailable)
{
  using pgo::Vec3d;
  using pgo::Vec3i;
  std::vector<Vec3d> vertices = {
    Vec3d(0.0, 0.0, 0.0),
    Vec3d(1.0, 0.0, 0.0),
    Vec3d(0.0, 1.0, 0.0),
    Vec3d(0.0, 0.0, 1.0),
  };
  std::vector<Vec3i> triangles = {
    Vec3i(0, 2, 1),
    Vec3i(0, 1, 3),
    Vec3i(1, 2, 3),
    Vec3i(2, 0, 3),
  };
  const pgo::Mesh::TriMeshGeo mesh(std::move(vertices), std::move(triangles));

  const auto smoothingStart = std::chrono::steady_clock::now();
  const pgo::Mesh::TriMeshGeo smoothed =
    pgo::CGALInterface::smoothMesh(mesh, 1, 180.0);
  const auto smoothingEnd = std::chrono::steady_clock::now();
  RecordProperty("dense_qr_smoothing_ms",
    std::chrono::duration<double, std::milli>(smoothingEnd - smoothingStart).count());
  EXPECT_EQ(smoothed.numVertices(), mesh.numVertices());
  EXPECT_EQ(smoothed.numTriangles(), mesh.numTriangles());

  bool allFixed = false;
  const auto repairStart = std::chrono::steady_clock::now();
  const pgo::Mesh::TriMeshGeo repaired =
    pgo::CGALInterface::repairSelfIntersections(mesh, "autorefine", &allFixed);
  const auto repairEnd = std::chrono::steady_clock::now();
  RecordProperty("self_intersection_repair_ms",
    std::chrono::duration<double, std::milli>(repairEnd - repairStart).count());
  EXPECT_TRUE(allFixed);
  EXPECT_GT(repaired.numVertices(), 0);
  EXPECT_GT(repaired.numTriangles(), 0);
}
