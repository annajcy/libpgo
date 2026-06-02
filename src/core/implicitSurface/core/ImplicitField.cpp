#include "core/ImplicitField.h"

#include "fields/GridField.h"

namespace pgo::ImplicitSurface {

Mesh::LightBoundingBox ImplicitField::bounds() const
{
  return Mesh::LightBoundingBox();
}

void ImplicitField::evalBatch(const V3d *pts, double *out, std::size_t n) const
{
  for (std::size_t i = 0; i < n; ++i)
    out[i] = eval(pts[i]);
}

GridField ImplicitField::sampleToGrid(const GridSpec &spec, int numThreads) const
{
  GridField grid(spec);
  const int resolution = spec.resolution;
  const V3d delta = (spec.bmax - spec.bmin) / static_cast<double>(resolution - 1);

  auto sample = [&](int x, int y, int z) {
    const V3d p = spec.bmin + delta.cwiseProduct(V3d(x, y, z).cast<double>());
    grid.at(x, y, z) = eval(p);
  };

  if (numThreads == 1) {
    for (int z = 0; z < resolution; ++z)
      for (int y = 0; y < resolution; ++y)
        for (int x = 0; x < resolution; ++x)
          sample(x, y, z);
    return grid;
  }

#ifdef USE_OPENMP
  if (numThreads > 1) {
#pragma omp parallel for collapse(3) num_threads(numThreads)
    for (int z = 0; z < resolution; ++z)
      for (int y = 0; y < resolution; ++y)
        for (int x = 0; x < resolution; ++x)
          sample(x, y, z);
  }
  else {
#pragma omp parallel for collapse(3)
    for (int z = 0; z < resolution; ++z)
      for (int y = 0; y < resolution; ++y)
        for (int x = 0; x < resolution; ++x)
          sample(x, y, z);
  }
#else
  for (int z = 0; z < resolution; ++z)
    for (int y = 0; y < resolution; ++y)
      for (int x = 0; x < resolution; ++x)
        sample(x, y, z);
#endif

  return grid;
}

}  // namespace pgo::ImplicitSurface
