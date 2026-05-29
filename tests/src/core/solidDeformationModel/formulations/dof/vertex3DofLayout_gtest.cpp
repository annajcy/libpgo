#include <gtest/gtest.h>

#include "formulations/dof/vertex3DofLayout.h"
#include "simulationMesh.h"
#include "triMeshGeo.h"

#include <memory>

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::SolidDeformationModel::SimulationMesh;
using pgo::SolidDeformationModel::SimulationMeshENuMaterial;
using pgo::SolidDeformationModel::SimulationMeshENuhMaterial;
using pgo::SolidDeformationModel::SimulationMeshMaterial;
using pgo::SolidDeformationModel::SimulationMeshType;
using pgo::SolidDeformationModel::Vertex3DofLayout;

constexpr const char *kShellObjPath = LIBPGO_TEST_SHELL_OBJ;

std::unique_ptr<SimulationMesh> makeSingleTetMesh()
{
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3 };
  const int elementMaterialIndices[] = { 0 };
  SimulationMeshENuMaterial mat(1000.0, 0.45);
  const SimulationMeshMaterial *materials[] = { &mat };
  return std::unique_ptr<SimulationMesh>(new SimulationMesh(
    4, vertices, 1, 4, elementVertices,
    elementMaterialIndices, 1, materials,
    SimulationMeshType::TET));
}

std::unique_ptr<SimulationMesh> makeSingleCubicMesh()
{
  const double vertices[] = {
    0.0, 0.0, 0.0,  1.0, 0.0, 0.0,
    1.0, 1.0, 0.0,  0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,  1.0, 0.0, 1.0,
    1.0, 1.0, 1.0,  0.0, 1.0, 1.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  const int elementMaterialIndices[] = { 0 };
  SimulationMeshENuMaterial mat(1000.0, 0.45);
  const SimulationMeshMaterial *materials[] = { &mat };
  return std::unique_ptr<SimulationMesh>(new SimulationMesh(
    8, vertices, 1, 8, elementVertices,
    elementMaterialIndices, 1, materials,
    SimulationMeshType::CUBIC));
}
}

TEST(Vertex3DofLayoutGTest, TetDofCounts)
{
  auto mesh = makeSingleTetMesh();
  Vertex3DofLayout layout(mesh.get());

  EXPECT_EQ(layout.numGlobalDofs(), 12);   // 4 vertices * 3
  EXPECT_EQ(layout.numLocalDofs(0), 12);   // 4 element vertices * 3
  EXPECT_EQ(layout.numLocalDofs(0), layout.numGlobalDofs());
}

TEST(Vertex3DofLayoutGTest, CubicDofCounts)
{
  auto mesh = makeSingleCubicMesh();
  Vertex3DofLayout layout(mesh.get());

  EXPECT_EQ(layout.numGlobalDofs(), 24);   // 8 vertices * 3
  EXPECT_EQ(layout.numLocalDofs(0), 24);   // 8 element vertices * 3
}

TEST(Vertex3DofLayoutGTest, ShellDofCounts)
{
  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(kShellObjPath));

  SimulationMeshENuhMaterial mat(1000.0, 0.45, 1e-3);
  auto mesh = pgo::SolidDeformationModel::loadShellMesh(surfaceMesh, &mat);
  ASSERT_NE(mesh, nullptr);

  Vertex3DofLayout layout(mesh.get());

  const int nv = mesh->getNumVertices();
  EXPECT_EQ(layout.numGlobalDofs(), nv * 3);
  EXPECT_EQ(layout.numLocalDofs(0), 18);   // 6 shell nodes * 3
  EXPECT_GT(layout.numGlobalDofs(), layout.numLocalDofs(0));
}

TEST(Vertex3DofLayoutGTest, GlobalDofIndices)
{
  auto mesh = makeSingleCubicMesh();
  Vertex3DofLayout layout(mesh.get());

  std::vector<int> indices;
  layout.getGlobalDofIndices(0, indices);
  ASSERT_EQ(static_cast<int>(indices.size()), 24);

  for (int j = 0; j < 8; j++) {
    int vid = mesh->getVertexIndex(0, j);
    for (int d = 0; d < 3; d++) {
      EXPECT_EQ(indices[j * 3 + d], vid * 3 + d);
    }
  }
}

TEST(Vertex3DofLayoutGTest, GatherAndScatterRoundTrip)
{
  auto mesh = makeSingleTetMesh();
  Vertex3DofLayout layout(mesh.get());

  const int nGlobal = layout.numGlobalDofs();
  const int nLocal = layout.numLocalDofs(0);

  // Set up global vector with known values
  std::vector<double> global(nGlobal, 0.0);
  for (int i = 0; i < nGlobal; i++)
    global[i] = static_cast<double>(i + 1);

  // Gather into local
  std::vector<double> local(nLocal);
  layout.gather(0, global.data(), local.data());

  // Scatter negative back
  std::vector<double> globalCopy = global;
  for (int i = 0; i < nLocal; i++)
    local[i] = -local[i];
  layout.scatterAddGradient(0, local.data(), globalCopy.data());

  // After scatter-add of negative, the DOFs touched should be zero
  // and untouched DOFs should remain unchanged
  std::vector<int> touchedDofs;
  layout.getGlobalDofIndices(0, touchedDofs);

  for (int i = 0; i < nGlobal; i++) {
    bool touched = false;
    for (int idx : touchedDofs) {
      if (idx == i) { touched = true; break; }
    }
    if (touched) {
      EXPECT_NEAR(globalCopy[i], 0.0, 1e-14);
    }
    else {
      EXPECT_DOUBLE_EQ(globalCopy[i], global[i]);
    }
  }
}

TEST(Vertex3DofLayoutGTest, HessianSparsityPattern)
{
  auto mesh = makeSingleTetMesh();
  Vertex3DofLayout layout(mesh.get());

  std::vector<ES::TripletD> entries;
  layout.addHessianSparsity(0, entries);

  // 4 vertices * 4 vertices * 3 * 3 = 144 triplets for a full block
  EXPECT_EQ(entries.size(), 144u);

  // Verify each triplet has valid row/col indices
  for (const auto &t : entries) {
    EXPECT_GE(t.row(), 0);
    EXPECT_LT(t.row(), 12);
    EXPECT_GE(t.col(), 0);
    EXPECT_LT(t.col(), 12);
  }
}

TEST(Vertex3DofLayoutGTest, BuildLocalToGlobalMatrixIndices)
{
  auto mesh = makeSingleTetMesh();
  Vertex3DofLayout layout(mesh.get());

  // Build a KTemplate for the mesh
  std::vector<ES::TripletD> entries;
  layout.addHessianSparsity(0, entries);

  ES::SpMatD KTemplate(12, 12);
  KTemplate.setFromTriplets(entries.begin(), entries.end());

  pgo::SolidDeformationModel::DynamicIndexMatrix indices;
  layout.buildLocalToGlobalMatrixIndices(0, KTemplate, indices);

  EXPECT_EQ(indices.rows(), 12);
  EXPECT_EQ(indices.cols(), 12);

  // Every local DOF pair that corresponds to valid vertices should have a non-negative offset
  for (int vi = 0; vi < 4; vi++) {
    int vidI = mesh->getVertexIndex(0, vi);
    for (int vj = 0; vj < 4; vj++) {
      int vidJ = mesh->getVertexIndex(0, vj);
      for (int dofi = 0; dofi < 3; dofi++) {
        for (int dofj = 0; dofj < 3; dofj++) {
          int localRow = vi * 3 + dofi;
          int localCol = vj * 3 + dofj;
          if (vidI >= 0 && vidJ >= 0) {
            EXPECT_GE(indices(localRow, localCol), 0);
          }
        }
      }
    }
  }
}

TEST(Vertex3DofLayoutGTest, GatherPreservesValues)
{
  auto mesh = makeSingleTetMesh();
  Vertex3DofLayout layout(mesh.get());

  const int nGlobal = layout.numGlobalDofs();
  const int nLocal = layout.numLocalDofs(0);

  std::vector<double> global(nGlobal, 0.0);
  for (int i = 0; i < nGlobal; i++)
    global[i] = static_cast<double>(i * 3 + 1);

  std::vector<double> local(nLocal);
  layout.gather(0, global.data(), local.data());

  // Verify the gathered values match the global positions
  for (int j = 0; j < 4; j++) {
    int vid = mesh->getVertexIndex(0, j);
    for (int d = 0; d < 3; d++) {
      EXPECT_DOUBLE_EQ(local[j * 3 + d], global[vid * 3 + d]);
    }
  }
}
