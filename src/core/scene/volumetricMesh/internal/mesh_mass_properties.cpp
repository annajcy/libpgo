#include "internal/mesh_mass_properties.h"

#include <cstring>

namespace pgo::VolumetricMeshes::internal::mesh_mass_properties {

double get_volume(const VolumetricMesh& mesh) {
    double volume = 0.0;
    for (int element = 0; element < mesh.getNumElements(); element++) {
        volume += mesh.getElementVolume(element);
    }
    return volume;
}

void get_vertex_volumes(const VolumetricMesh& mesh, double* vertex_volumes) {
    std::memset(vertex_volumes, 0, sizeof(double) * mesh.getNumVertices());
    const double factor = 1.0 / mesh.getNumElementVertices();
    for (int element = 0; element < mesh.getNumElements(); element++) {
        const double volume = mesh.getElementVolume(element);
        for (int vertex = 0; vertex < mesh.getNumElementVertices(); vertex++) {
            vertex_volumes[mesh.getVertexIndex(element, vertex)] += factor * volume;
        }
    }
}

double get_mass(const VolumetricMesh& mesh) {
    double mass = 0.0;
    for (int region_index = 0; region_index < mesh.getNumRegions(); region_index++) {
        const MaterialRegion& region = mesh.getRegion(region_index);
        const double density = material_density(mesh.getMaterial(region.getMaterialIndex()));
        std::set<int> set_elements;
        mesh.getSet(region.getSetIndex()).getElements(set_elements);

        for (std::set<int>::iterator iter = set_elements.begin(); iter != set_elements.end(); iter++) {
            const int element = *iter;
            mass += mesh.getElementVolume(element) * density;
        }
    }

    return mass;
}

void get_inertia_parameters(const VolumetricMesh& mesh, double& mass, Vec3d& center_of_mass, Mat3d& inertia_tensor) {
    mass = 0.0;
    center_of_mass.setZero();
    inertia_tensor.setZero();

    for (int element = 0; element < mesh.getNumElements(); element++) {
        const double density = mesh.getElementDensity(element);
        const double element_volume = mesh.getElementVolume(element);
        const double element_mass = element_volume * density;

        mass += element_mass;
        const Vec3d element_center = mesh.getElementCenter(element);
        center_of_mass += element_mass * element_center;

        Mat3d element_it_unit_density;
        mesh.getElementInertiaTensor(element, element_it_unit_density);

        const double a = element_center[0];
        const double b = element_center[1];
        const double c = element_center[2];

        const Mat3d element_it_correction = asMat3d(b * b + c * c, -a * b, -a * c, -a * b, a * a + c * c, -b * c,
                                                    -a * c, -b * c, a * a + b * b);

        inertia_tensor += density * element_it_unit_density + element_mass * element_it_correction;
    }

    center_of_mass /= mass;

    const double a = center_of_mass[0];
    const double b = center_of_mass[1];
    const double c = center_of_mass[2];
    const Mat3d correction =
        asMat3d(b * b + c * c, -a * b, -a * c, -a * b, a * a + c * c, -b * c, -a * c, -b * c, a * a + b * b);

    inertia_tensor -= mass * correction;
}

void get_mesh_geometric_parameters(const VolumetricMesh& mesh, Vec3d& centroid, double* radius) {
    centroid = Vec3d(0, 0, 0);
    for (int i = 0; i < mesh.getNumVertices(); i++) {
        centroid += mesh.getVertex(i);
    }

    centroid /= mesh.getNumVertices();

    *radius = 0;
    for (int i = 0; i < mesh.getNumVertices(); i++) {
        const Vec3d vertex = mesh.getVertex(i);
        const double dist = (vertex - centroid).norm();
        if (dist > *radius) {
            *radius = dist;
        }
    }
}

Mesh::BoundingBox get_bounding_box(const VolumetricMesh& mesh) {
    const auto vertices = mesh.getVertices();
    return Mesh::BoundingBox(std::vector<Vec3d>(vertices.begin(), vertices.end()));
}

}  // namespace pgo::VolumetricMeshes::internal::mesh_mass_properties
