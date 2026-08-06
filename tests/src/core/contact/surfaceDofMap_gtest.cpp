#include <gtest/gtest.h>

#include "surfaceDofMap.h"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <string_view>
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

ES::SpMatD makeDuplicateColumnSurfaceMap()
{
  std::vector<ES::TripletD> entries;
  entries.emplace_back(0, 0, 1.0);
  entries.emplace_back(0, 2, 0.5);
  entries.emplace_back(1, 0, 2.0);
  entries.emplace_back(1, 2, -0.25);
  entries.emplace_back(2, 1, 1.5);
  entries.emplace_back(3, 1, -1.0);
  entries.emplace_back(4, 2, 2.0);
  entries.emplace_back(5, 3, 1.0);
  ES::SpMatD W(6, 5);
  W.setFromTriplets(entries.begin(), entries.end());
  return W;
}

ES::SpMatD makeDuplicateColumnSurfaceHessian()
{
  std::vector<ES::TripletD> entries;
  entries.emplace_back(0, 0, 1.0);
  entries.emplace_back(0, 1, 2.0);
  entries.emplace_back(1, 0, 3.0);
  entries.emplace_back(1, 1, 4.0);
  entries.emplace_back(2, 2, 5.0);
  entries.emplace_back(3, 2, -0.5);
  entries.emplace_back(4, 4, 6.0);
  entries.emplace_back(5, 5, 7.0);
  ES::SpMatD H(6, 6);
  H.setFromTriplets(entries.begin(), entries.end());
  return H;
}


std::uint64_t countTmpStreams(const ES::SpMatD &W, const ES::SpMatD &tmp)
{
  std::uint64_t streamCount = 0;
  for (Eigen::Index surfaceRow = 0; surfaceRow < W.rows(); ++surfaceRow) {
    const Eigen::Index tmpRowNnz = tmp.outerIndexPtr()[surfaceRow + 1] - tmp.outerIndexPtr()[surfaceRow];
    if (tmpRowNnz == 0)
      continue;

    for (ES::SpMatD::InnerIterator it(W, surfaceRow); it; ++it)
      ++streamCount;
  }
  return streamCount;
}

std::uint64_t countConceptualTmpContributions(const ES::SpMatD &W, const ES::SpMatD &tmp)
{
  std::uint64_t contributionCount = 0;
  for (Eigen::Index surfaceRow = 0; surfaceRow < W.rows(); ++surfaceRow) {
    const std::uint64_t tmpRowNnz =
      static_cast<std::uint64_t>(tmp.outerIndexPtr()[surfaceRow + 1] - tmp.outerIndexPtr()[surfaceRow]);
    for (ES::SpMatD::InnerIterator it(W, surfaceRow); it; ++it)
      contributionCount += tmpRowNnz;
  }
  return contributionCount;
}

void runCustomPullbackDuplicateColumnCase()
{
  const ES::SpMatD W = makeDuplicateColumnSurfaceMap();
  const ES::SpMatD surfaceHessian = makeDuplicateColumnSurfaceHessian();
  Contact::SurfaceDofMap map(makeRestVertices(), W);


  ES::SpMatD simulationHessian;
  map.pullbackHessian(surfaceHessian, simulationHessian);

  const ES::SpMatD expected = W.transpose() * surfaceHessian * W;
  EXPECT_EQ(simulationHessian.rows(), expected.rows());
  EXPECT_EQ(simulationHessian.cols(), expected.cols());
  EXPECT_TRUE(simulationHessian.isCompressed());
  EXPECT_TRUE(ES::MXd(simulationHessian).isApprox(ES::MXd(expected), 1e-12));

  ES::SpMatD tmp = surfaceHessian * W;
  tmp.makeCompressed();
  EXPECT_GT(countConceptualTmpContributions(W, tmp), static_cast<std::uint64_t>(simulationHessian.nonZeros()));
  EXPECT_EQ(simulationHessian.outerIndexPtr()[4 + 1] - simulationHessian.outerIndexPtr()[4], 0);

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


TEST(SurfaceDofMapGTest, PullbackDirectFillMatchesEigenAndReducesDuplicateColumns)
{
  runCustomPullbackDuplicateColumnCase();
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
