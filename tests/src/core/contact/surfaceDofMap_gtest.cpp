#include <gtest/gtest.h>

#include "surfaceDofMap.h"

#include <stdexcept>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
namespace Contact = pgo::Contact;

ES::MXd makeRestVertices()
{
  ES::MXd V(2, 3);
  V << 1.0, 2.0, 3.0,
       4.0, 5.0, 6.0;
  return V;
}

ES::SpMatD makeSurfaceMap()
{
  std::vector<ES::TripletD> entries;
  entries.emplace_back(0, 0, 1.0);
  entries.emplace_back(1, 1, 1.0);
  entries.emplace_back(2, 2, 1.0);
  entries.emplace_back(3, 3, 2.0);
  entries.emplace_back(4, 4, 3.0);
  entries.emplace_back(5, 5, 4.0);
  entries.emplace_back(0, 6, 0.5);
  ES::SpMatD W(6, 7);
  W.setFromTriplets(entries.begin(), entries.end());
  return W;
}

ES::SpMatD makeSurfaceHessian()
{
  std::vector<ES::TripletD> entries;
  entries.emplace_back(0, 0, 1.0);
  entries.emplace_back(0, 3, 0.25);
  entries.emplace_back(3, 0, 0.25);
  entries.emplace_back(1, 1, 2.0);
  entries.emplace_back(2, 2, 3.0);
  entries.emplace_back(3, 3, 4.0);
  entries.emplace_back(4, 4, 5.0);
  entries.emplace_back(5, 5, 6.0);
  ES::SpMatD H(6, 6);
  H.setFromTriplets(entries.begin(), entries.end());
  return H;
}
}  // namespace

TEST(SurfaceDofMapGTest, MapsSimulationDisplacementsAndAbsolutePositions)
{
  const ES::MXd rest = makeRestVertices();
  const ES::SpMatD W = makeSurfaceMap();
  Contact::SurfaceDofMap map(rest, W);

  ES::VXd u(7);
  u << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8;

  ES::VXd expectedDisplacements(6);
  expectedDisplacements << 0.5, 0.2, 0.3, 0.8, 1.5, 2.4;

  ES::VXd expectedPositions(6);
  expectedPositions << 1.5, 2.2, 3.3, 4.8, 6.5, 8.4;

  EXPECT_TRUE(map.surfaceDisplacements(u).isApprox(expectedDisplacements, 1e-12));
  EXPECT_TRUE(map.surfacePositions(u).isApprox(expectedPositions, 1e-12));
}

TEST(SurfaceDofMapGTest, ReportsSimulationAndSurfaceDofCounts)
{
  Contact::SurfaceDofMap map(makeRestVertices(), makeSurfaceMap());

  EXPECT_EQ(map.numSimulationDofs(), 7);
  EXPECT_EQ(map.numSurfaceDofs(), 6);
  EXPECT_EQ(map.simulationDofs(), (std::vector<int>{ 0, 1, 2, 3, 4, 5, 6 }));
}

TEST(SurfaceDofMapGTest, PullsBackSurfaceGradient)
{
  const ES::SpMatD W = makeSurfaceMap();
  Contact::SurfaceDofMap map(makeRestVertices(), W);

  ES::VXd surfaceGradient(6);
  surfaceGradient << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  const ES::VXd expected = W.transpose() * surfaceGradient;
  EXPECT_TRUE(map.pullbackGradient(surfaceGradient).isApprox(expected, 1e-12));
}

TEST(SurfaceDofMapGTest, PullsBackSurfaceHessian)
{
  const ES::SpMatD W = makeSurfaceMap();
  const ES::SpMatD surfaceHessian = makeSurfaceHessian();
  Contact::SurfaceDofMap map(makeRestVertices(), W);

  ES::SpMatD simulationHessian;
  map.pullbackHessian(surfaceHessian, simulationHessian);

  const ES::SpMatD expected = W.transpose() * surfaceHessian * W;
  EXPECT_TRUE(ES::MXd(simulationHessian).isApprox(ES::MXd(expected), 1e-12));
}

TEST(SurfaceDofMapGTest, ThrowsForInvalidRestVertexShape)
{
  ES::MXd rest(2, 2);
  rest.setZero();
  const ES::SpMatD W(6, 7);

  EXPECT_THROW(Contact::SurfaceDofMap(rest, W), std::invalid_argument);
}

TEST(SurfaceDofMapGTest, ThrowsForEmptyRestVertices)
{
  ES::MXd rest(0, 3);
  const ES::SpMatD W(0, 7);

  EXPECT_THROW(Contact::SurfaceDofMap(rest, W), std::invalid_argument);
}

TEST(SurfaceDofMapGTest, ThrowsForInvalidMapRowCount)
{
  ES::SpMatD W(5, 7);
  EXPECT_THROW(Contact::SurfaceDofMap(makeRestVertices(), W), std::invalid_argument);
}

TEST(SurfaceDofMapGTest, ThrowsForMapWithNoSimulationDofs)
{
  const ES::SpMatD W(6, 0);

  EXPECT_THROW(Contact::SurfaceDofMap(makeRestVertices(), W), std::invalid_argument);
}

TEST(SurfaceDofMapGTest, ThrowsForInvalidSimulationDisplacementSize)
{
  Contact::SurfaceDofMap map(makeRestVertices(), makeSurfaceMap());

  ES::VXd u = ES::VXd::Zero(6);

  EXPECT_THROW((void)map.surfaceDisplacements(u), std::invalid_argument);
  EXPECT_THROW((void)map.surfacePositions(u), std::invalid_argument);
}

TEST(SurfaceDofMapGTest, ThrowsForInvalidSurfaceGradientSize)
{
  Contact::SurfaceDofMap map(makeRestVertices(), makeSurfaceMap());

  ES::VXd surfaceGradient = ES::VXd::Zero(5);

  EXPECT_THROW((void)map.pullbackGradient(surfaceGradient), std::invalid_argument);
}

TEST(SurfaceDofMapGTest, ThrowsForInvalidSurfaceHessianShape)
{
  Contact::SurfaceDofMap map(makeRestVertices(), makeSurfaceMap());

  ES::SpMatD wrongRows(5, 6);
  ES::SpMatD wrongColumns(6, 5);
  ES::SpMatD nonSquare(4, 6);
  ES::SpMatD simulationHessian;

  EXPECT_THROW(map.pullbackHessian(wrongRows, simulationHessian), std::invalid_argument);
  EXPECT_THROW(map.pullbackHessian(wrongColumns, simulationHessian), std::invalid_argument);
  EXPECT_THROW(map.pullbackHessian(nonSquare, simulationHessian), std::invalid_argument);
}
