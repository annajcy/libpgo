#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace pgo::benchmark_helpers
{

enum class RepresentativeWorkload
{
  NoBlas,
  FemElements,
  ContactPairs,
};

inline double runNoBlasWork(int taskIndex, int workItems)
{
  double sum = 0.0;
  for (int i = 0; i < workItems; ++i) {
    const double x = 0.0001 * static_cast<double>(1 + i + taskIndex * 17);
    sum += (x * x + 0.25 * x + 1.0) / (1.0 + 0.125 * x);
  }
  return sum;
}

inline double runFemElementWork(int taskIndex, int workItems)
{
  double sum = 0.0;
  for (int i = 0; i < workItems; ++i) {
    const double s = 0.00001 * static_cast<double>(1 + i + taskIndex * 31);
    const std::array<double, 9> f = {
      1.0 + s, 0.10 + s * 0.5, -0.04,
      0.02, 1.0 - s * 0.25, 0.06 + s,
      -0.03 + s * 0.2, 0.01, 1.0 + s * 0.75
    };
    const double det = f[0] * (f[4] * f[8] - f[5] * f[7]) -
      f[1] * (f[3] * f[8] - f[5] * f[6]) +
      f[2] * (f[3] * f[7] - f[4] * f[6]);
    double frobenius = 0.0;
    for (double value : f)
      frobenius += value * value;
    const double logJ = std::log(det);
    sum += 0.5 * (frobenius - 3.0) - logJ + 2.5 * logJ * logJ;
  }
  return sum;
}

inline double runContactPairWork(int taskIndex, int workItems)
{
  double sum = 0.0;
  for (int i = 0; i < workItems; ++i) {
    const double s = 0.00001 * static_cast<double>(1 + i + taskIndex * 13);
    const std::array<double, 3> edge0 = { 1.0 + s, 0.2, -0.1 };
    const std::array<double, 3> edge1 = { -0.15, 0.9 + 0.5 * s, 0.3 };
    const std::array<double, 3> point = { 0.2 + s, 0.25 - 0.2 * s, 0.05 };
    const std::array<double, 3> normal = {
      edge0[1] * edge1[2] - edge0[2] * edge1[1],
      edge0[2] * edge1[0] - edge0[0] * edge1[2],
      edge0[0] * edge1[1] - edge0[1] * edge1[0]
    };
    const double normalSquared =
      normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2];
    const double signedNumerator =
      point[0] * normal[0] + point[1] * normal[1] + point[2] * normal[2];
    const double distanceSquared = signedNumerator * signedNumerator / normalSquared;
    const double gap = std::max(1e-8, 0.08 - distanceSquared);
    sum += gap * gap * (-std::log(gap / 0.08));
  }
  return sum;
}

inline double runRepresentativeWorkload(
  RepresentativeWorkload workload, int taskIndex, int workItems)
{
  switch (workload) {
  case RepresentativeWorkload::NoBlas:
    return runNoBlasWork(taskIndex, workItems);
  case RepresentativeWorkload::FemElements:
    return runFemElementWork(taskIndex, workItems);
  case RepresentativeWorkload::ContactPairs:
    return runContactPairWork(taskIndex, workItems);
  }
  return 0.0;
}

inline const char *workloadName(RepresentativeWorkload workload)
{
  switch (workload) {
  case RepresentativeWorkload::NoBlas:
    return "NoBlasControl";
  case RepresentativeWorkload::FemElements:
    return "FemElementBatch";
  case RepresentativeWorkload::ContactPairs:
    return "ContactPairBatch";
  }
  return "Unknown";
}

}  // namespace pgo::benchmark_helpers
