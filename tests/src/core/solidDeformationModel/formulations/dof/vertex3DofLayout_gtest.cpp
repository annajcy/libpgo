#include <gtest/gtest.h>

#include "formulations/dof/perVertexDofLayout.h"
#include "formulations/dof/vertex3DofLayout.h"
#include "simulation/simulationMesh.h"
#include "triMeshGeo.h"

#include <memory>
#include <vector>

namespace
{
using pgo::SolidDeformationModel::DofGroup;
using pgo::SolidDeformationModel::SimulationMesh;
using pgo::SolidDeformationModel::SimulationMeshENuMaterial;
using pgo::SolidDeformationModel::SimulationMeshENuhMaterial;
using pgo::SolidDeformationModel::SimulationMeshType;
using pgo::SolidDeformationModel::PerVertexDofLayout;
using pgo::SolidDeformationModel::Vertex3DofLayout;

constexpr const char *kShellObjPath = LIBPGO_TEST_SHELL_OBJ;

class GroupOnlyLayout : public pgo::SolidDeformationModel::DofLayout
{
public:
  int numGlobalDofs() const override { return 8; }
  int numLocalDofs(int) const override { return 6; }

  void getDofGroups(int, std::vector<DofGroup> &groups) const override
  {
    groups = {
      DofGroup{ 0, 2, 2 },
      DofGroup{ 4, 6, 2 },
    };
  }
};

std::unique_ptr<SimulationMesh> makeSingleTetMesh()
{
  const double vertices[] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  const int elementVertices[] = { 0, 1, 2, 3 };
  SimulationMeshENuMaterial mat(1000.0, 0.45);
  return std::unique_ptr<SimulationMesh>(new SimulationMesh(
    4, vertices, 1, 4, elementVertices,
    makeUniformSimulationMeshElementFieldStore(1, mat),
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
  SimulationMeshENuMaterial mat(1000.0, 0.45);
  return std::unique_ptr<SimulationMesh>(new SimulationMesh(
    8, vertices, 1, 8, elementVertices,
    makeUniformSimulationMeshElementFieldStore(1, mat),
    SimulationMeshType::CUBIC));
}
}

TEST(DofLayoutGTest, DefaultGatherAndScatterUseGroups)
{
  GroupOnlyLayout layout;
  const std::vector<double> global = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0 };
  std::vector<double> local(layout.numLocalDofs(0), -1.0);
  std::vector<DofGroup> groups;

  layout.gather(0, global, local, groups);

  EXPECT_DOUBLE_EQ(local[0], 3.0);
  EXPECT_DOUBLE_EQ(local[1], 4.0);
  EXPECT_DOUBLE_EQ(local[2], 0.0);
  EXPECT_DOUBLE_EQ(local[3], 0.0);
  EXPECT_DOUBLE_EQ(local[4], 7.0);
  EXPECT_DOUBLE_EQ(local[5], 8.0);

  std::vector<double> out(layout.numGlobalDofs(), 10.0);
  layout.scatterAddGradient(0, local, out, groups);

  EXPECT_DOUBLE_EQ(out[0], 10.0);
  EXPECT_DOUBLE_EQ(out[1], 10.0);
  EXPECT_DOUBLE_EQ(out[2], 13.0);
  EXPECT_DOUBLE_EQ(out[3], 14.0);
  EXPECT_DOUBLE_EQ(out[4], 10.0);
  EXPECT_DOUBLE_EQ(out[5], 10.0);
  EXPECT_DOUBLE_EQ(out[6], 17.0);
  EXPECT_DOUBLE_EQ(out[7], 18.0);
}

TEST(PerVertexDofLayoutGTest, MapsEachElementVertexToContiguousDofGroup)
{
  auto mesh = makeSingleTetMesh();
  PerVertexDofLayout<5> layout(*mesh);

  EXPECT_EQ(layout.numGlobalDofs(), 20);
  EXPECT_EQ(layout.numLocalDofs(0), 20);

  std::vector<DofGroup> groups;
  layout.getDofGroups(0, groups);
  ASSERT_EQ(groups.size(), 4u);

  for (int v = 0; v < 4; v++) {
    const int vid = mesh->getVertexIndex(0, v);
    EXPECT_EQ(groups[v].localStart, v * 5);
    EXPECT_EQ(groups[v].globalStart, vid * 5);
    EXPECT_EQ(groups[v].size, 5);
  }
}

TEST(Vertex3DofLayoutGTest, TetDofCounts)
{
  auto mesh = makeSingleTetMesh();
  Vertex3DofLayout layout(*mesh);

  EXPECT_EQ(layout.numGlobalDofs(), 12);   // 4 vertices * 3
  EXPECT_EQ(layout.numLocalDofs(0), 12);   // 4 element vertices * 3
  EXPECT_EQ(layout.numLocalDofs(0), layout.numGlobalDofs());
}

TEST(Vertex3DofLayoutGTest, CubicDofCounts)
{
  auto mesh = makeSingleCubicMesh();
  Vertex3DofLayout layout(*mesh);

  EXPECT_EQ(layout.numGlobalDofs(), 24);   // 8 vertices * 3
  EXPECT_EQ(layout.numLocalDofs(0), 24);   // 8 element vertices * 3
}

TEST(Vertex3DofLayoutGTest, ShellDofCounts)
{
  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(kShellObjPath));

  SimulationMeshENuhMaterial mat(1000.0, 0.45, 1e-3);
  auto mesh = pgo::SolidDeformationModel::loadShellMesh(surfaceMesh, mat);
  ASSERT_NE(mesh, nullptr);

  Vertex3DofLayout layout(*mesh);

  const int nv = mesh->getNumVertices();
  EXPECT_EQ(layout.numGlobalDofs(), nv * 3);
  EXPECT_EQ(layout.numLocalDofs(0), 18);   // 6 shell nodes * 3
  EXPECT_GT(layout.numGlobalDofs(), layout.numLocalDofs(0));
}

TEST(Vertex3DofLayoutGTest, GatherAndScatterRoundTrip)
{
  auto mesh = makeSingleTetMesh();
  Vertex3DofLayout layout(*mesh);

  const int nGlobal = layout.numGlobalDofs();
  const int nLocal = layout.numLocalDofs(0);

  // Set up global vector with known values
  std::vector<double> global(nGlobal, 0.0);
  for (int i = 0; i < nGlobal; i++)
    global[i] = static_cast<double>(i + 1);

  // Gather into local
  std::vector<double> local(nLocal);
  std::vector<DofGroup> groups;
  layout.gather(0, global, local, groups);

  // Scatter negative back
  std::vector<double> globalCopy = global;
  for (int i = 0; i < nLocal; i++)
    local[i] = -local[i];
  layout.scatterAddGradient(0, local, globalCopy, groups);

  // After scatter-add of negative, the DOFs touched should be zero
  // and untouched DOFs should remain unchanged
  layout.getDofGroups(0, groups);

  for (int i = 0; i < nGlobal; i++) {
    bool touched = false;
    for (const DofGroup &group : groups) {
      if (i >= group.globalStart && i < group.globalStart + group.size) {
        touched = true;
        break;
      }
    }
    if (touched) {
      EXPECT_NEAR(globalCopy[i], 0.0, 1e-14);
    }
    else {
      EXPECT_DOUBLE_EQ(globalCopy[i], global[i]);
    }
  }
}

TEST(Vertex3DofLayoutGTest, DofGroupsForTet)
{
  auto mesh = makeSingleTetMesh();
  Vertex3DofLayout layout(*mesh);

  std::vector<DofGroup> groups;
  layout.getDofGroups(0, groups);
  ASSERT_EQ(groups.size(), 4u);

  for (int v = 0; v < 4; v++) {
    const int vid = mesh->getVertexIndex(0, v);
    EXPECT_EQ(groups[v].localStart, v * 3);
    EXPECT_EQ(groups[v].globalStart, vid * 3);
    EXPECT_EQ(groups[v].size, 3);
    EXPECT_EQ(groups[v].globalDof(0), vid * 3 + 0);
    EXPECT_EQ(groups[v].globalDof(1), vid * 3 + 1);
    EXPECT_EQ(groups[v].globalDof(2), vid * 3 + 2);
  }
}

TEST(Vertex3DofLayoutGTest, DofGroupsSkipShellSentinels)
{
  pgo::Mesh::TriMeshGeo surfaceMesh;
  ASSERT_TRUE(surfaceMesh.load(kShellObjPath));

  SimulationMeshENuhMaterial mat(1000.0, 0.45, 1e-3);
  auto mesh = pgo::SolidDeformationModel::loadShellMesh(surfaceMesh, mat);
  ASSERT_NE(mesh, nullptr);

  Vertex3DofLayout layout(*mesh);

  std::vector<DofGroup> groups;
  layout.getDofGroups(0, groups);

  int expectedGroup = 0;
  int sentinelCount = 0;
  for (int v = 0; v < mesh->getNumElementVertices(); v++) {
    const int vid = mesh->getVertexIndex(0, v);
    if (vid < 0) {
      sentinelCount++;
      continue;
    }

    ASSERT_LT(expectedGroup, static_cast<int>(groups.size()));
    const DofGroup &group = groups[expectedGroup++];
    EXPECT_EQ(group.localStart, v * 3);
    EXPECT_EQ(group.globalStart, vid * 3);
    EXPECT_EQ(group.size, 3);
    EXPECT_EQ(group.globalDof(0), vid * 3 + 0);
    EXPECT_EQ(group.globalDof(1), vid * 3 + 1);
    EXPECT_EQ(group.globalDof(2), vid * 3 + 2);
  }

  EXPECT_GT(sentinelCount, 0);
  EXPECT_EQ(expectedGroup, static_cast<int>(groups.size()));
}

TEST(Vertex3DofLayoutGTest, GatherPreservesValues)
{
  auto mesh = makeSingleTetMesh();
  Vertex3DofLayout layout(*mesh);

  const int nGlobal = layout.numGlobalDofs();
  const int nLocal = layout.numLocalDofs(0);

  std::vector<double> global(nGlobal, 0.0);
  for (int i = 0; i < nGlobal; i++)
    global[i] = static_cast<double>(i * 3 + 1);

  std::vector<double> local(nLocal);
  std::vector<DofGroup> groups;
  layout.gather(0, global, local, groups);

  // Verify the gathered values match the global positions
  for (int j = 0; j < 4; j++) {
    int vid = mesh->getVertexIndex(0, j);
    for (int d = 0; d < 3; d++) {
      EXPECT_DOUBLE_EQ(local[j * 3 + d], global[vid * 3 + d]);
    }
  }
}
