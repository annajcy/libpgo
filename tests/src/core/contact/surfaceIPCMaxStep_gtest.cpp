#include <gtest/gtest.h>

#include "ipc/core/surfaceIPCMaxStep.h"
#include "ipc/topology/surfaceIPCTopology.h"

#include "testCIPCHelpers.h"

#include <algorithm>
#include <string_view>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::Contact::IPC::SurfaceIPCTopology;
using pgo::Contact::CIPCTest::flattenPositions;
using pgo::Contact::CIPCTest::makeTwoTriangleMesh;

}  // namespace

TEST(SurfaceIPCMaxStepGTest, HelperComputesClampedSelfMaxStep)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd x = flattenPositions(V);
  ES::VXd dx = ES::VXd::Zero(x.size());
  for (int vi = 3; vi < 6; ++vi)
    dx[3 * vi + 2] = -0.1;

  SurfaceIPCTopology topology;
  topology.setMesh(V, F);

  const double helperAlpha = computeSelfMaxStep(topology, x, dx, 0.1, 0.9);

  EXPECT_GT(helperAlpha, 0.0);
  EXPECT_LT(helperAlpha, 1.0);
}

