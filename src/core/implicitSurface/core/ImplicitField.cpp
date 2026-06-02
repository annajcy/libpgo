#include "core/ImplicitField.h"

#include "fields/GridField.h"
#include "parallelism/parallelFor.h"

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

  pgo::parallel::parallelFor3D(resolution, resolution, resolution,
    { .numThreads = numThreads },
    [&](int x, int y, int z) {
      const V3d p = spec.bmin + delta.cwiseProduct(V3d(x, y, z).cast<double>());
      grid.at(x, y, z) = eval(p);
    });

  return grid;
}

}  // namespace pgo::ImplicitSurface
