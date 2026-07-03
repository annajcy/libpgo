#include "fields/SphereField.h"

#include "boundingBox.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace pgo::ImplicitSurface {

SphereField::SphereField(const V3d &center_, double radius_)
  : center(center_), radius(radius_)
{
  if (radius <= 0.0 || !std::isfinite(radius))
    throw std::runtime_error("SphereField radius must be positive");
}

double SphereField::eval(const V3d &p) const
{
  return (p - center).norm() - radius;
}

Mesh::LightBoundingBox SphereField::bounds() const
{
  const V3d r(radius, radius, radius);
  return Mesh::LightBoundingBox(center - r, center + r);
}

SphereField SphereField::fromMeshBBox(const Mesh::TriMeshGeo &mesh)
{
  if (mesh.numVertices() == 0)
    throw std::runtime_error("Cannot derive sphere parameters from an empty sphere mesh");

  const Mesh::BoundingBox bbox(mesh.positions());
  SphereField field;
  field.center = bbox.center();
  field.radius = 0.0;
  for (int i = 0; i < mesh.numVertices(); ++i)
    field.radius = std::max(field.radius, (mesh.pos(i) - field.center).norm());

  if (field.radius <= 0.0 || !std::isfinite(field.radius))
    throw std::runtime_error("Derived sphere radius is not positive");

  return field;
}

int SphereField::projectOpenBoundaryToSphere(Mesh::TriMeshGeo &mesh) const
{
  std::map<std::pair<int, int>, int> edgeIncidentCounts;
  for (int triID = 0; triID < mesh.numTriangles(); ++triID) {
    const Vec3i &tri = mesh.tri(triID);
    for (int i = 0; i < 3; ++i) {
      int a = tri[i];
      int b = tri[(i + 1) % 3];
      if (a > b)
        std::swap(a, b);
      ++edgeIncidentCounts[std::make_pair(a, b)];
    }
  }

  std::vector<char> isBoundaryVertex(mesh.numVertices(), 0);
  for (const auto &entry : edgeIncidentCounts) {
    if (entry.second != 1)
      continue;
    isBoundaryVertex[entry.first.first] = 1;
    isBoundaryVertex[entry.first.second] = 1;
  }

  int projectedVertices = 0;
  for (int vertexID = 0; vertexID < mesh.numVertices(); ++vertexID) {
    if (!isBoundaryVertex[vertexID])
      continue;

    const V3d radial = mesh.pos(vertexID) - center;
    const double radialNorm = radial.norm();
    if (radialNorm <= 0.0)
      continue;

    mesh.pos(vertexID) = center + radial * (radius / radialNorm);
    ++projectedVertices;
  }

  return projectedVertices;
}

}  // namespace pgo::ImplicitSurface
