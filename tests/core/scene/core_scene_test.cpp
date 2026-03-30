#include "boundingVolumeTree.h"
#include "geometryQuery.h"
#include "triMeshGeo.h"
#include "triMeshNeighbor.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace pgo::Mesh::test {

TEST(CoreSceneGeometryTest, TriangleAreaNormalAndCenter) {
    const pgo::Vec3d p0(0.0, 0.0, 0.0);
    const pgo::Vec3d p1(1.0, 0.0, 0.0);
    const pgo::Vec3d p2(0.0, 1.0, 0.0);

    EXPECT_NEAR(pgo::Mesh::getTriangleArea(p0, p1, p2), 0.5, 1e-12);

    const pgo::Vec3d n = pgo::Mesh::getTriangleNormal(p0, p1, p2);
    EXPECT_NEAR(n[0], 0.0, 1e-12);
    EXPECT_NEAR(n[1], 0.0, 1e-12);
    EXPECT_NEAR(n[2], 1.0, 1e-12);

    const pgo::Vec3d c = pgo::Mesh::getTriangleCenterOfMass(p0, p1, p2);
    EXPECT_NEAR(c[0], 1.0 / 3.0, 1e-12);
    EXPECT_NEAR(c[1], 1.0 / 3.0, 1e-12);
    EXPECT_NEAR(c[2], 0.0, 1e-12);
}

TEST(CoreSceneGeometryTest, TriMeshNeighborSharedEdge) {
    std::vector<pgo::Vec3d> vertices{pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(1.0, 1.0, 0.0),
                                     pgo::Vec3d(0.0, 1.0, 0.0)};

    std::vector<pgo::Vec3i> triangles{pgo::Vec3i(0, 1, 2), pgo::Vec3i(0, 2, 3)};

    pgo::Mesh::TriMeshGeo      mesh(std::move(vertices), std::move(triangles));
    pgo::Mesh::TriMeshNeighbor neighbor(mesh);

    const pgo::Vec3i n0          = neighbor.getTriangleNeighbors(0);
    int              sharedCount = 0;
    bool             hasTri1     = false;
    for (int i = 0; i < 3; i++) {
        if (n0[i] >= 0) {
            sharedCount++;
            if (n0[i] == 1) {
                hasTri1 = true;
            }
        }
    }

    EXPECT_EQ(sharedCount, 1);
    EXPECT_TRUE(hasTri1);
}

TEST(CoreSceneGeometryTest, TriMeshBVTreeClosestTriangle) {
    std::vector<pgo::Vec3d> vertices{pgo::Vec3d(0.0, 0.0, 0.0), pgo::Vec3d(1.0, 0.0, 0.0), pgo::Vec3d(1.0, 1.0, 0.0),
                                     pgo::Vec3d(0.0, 1.0, 0.0)};

    std::vector<pgo::Vec3i> triangles{pgo::Vec3i(0, 1, 2), pgo::Vec3i(0, 2, 3)};

    pgo::Mesh::TriMeshGeo    mesh(std::move(vertices), std::move(triangles));
    pgo::Mesh::TriMeshBVTree bvTree;
    bvTree.buildByInertiaPartition(mesh);

    const auto result = bvTree.closestTriangleQuery(mesh, pgo::Vec3d(0.2, 0.2, 0.8));
    EXPECT_GE(result.triID, 0);
    EXPECT_LT(result.triID, mesh.numTriangles());
    EXPECT_TRUE(std::isfinite(result.dist2));
    EXPECT_NEAR(result.closestPosition[2], 0.0, 1e-12);
}

}  // namespace pgo::Mesh::test
