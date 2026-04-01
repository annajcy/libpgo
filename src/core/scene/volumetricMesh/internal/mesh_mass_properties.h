#pragma once

#include "concepts/mesh_concepts.h"
#include "volumetricMesh.h"

#include <cstring>
#include <set>
#include <vector>

namespace pgo::VolumetricMeshes::internal::mesh_mass_properties {

namespace detail {

template <class MeshT>
double get_volume_impl(const MeshT& mesh) {
    double volume = 0.0;
    for (int element = 0; element < mesh.getNumElements(); element++) {
        volume += mesh.getElementVolume(element);
    }
    return volume;
}

template <class MeshT>
void get_vertex_volumes_impl(const MeshT& mesh, double* vertex_volumes) {
    std::memset(vertex_volumes, 0, sizeof(double) * mesh.getNumVertices());
    const double factor = 1.0 / mesh.getNumElementVertices();
    for (int element = 0; element < mesh.getNumElements(); element++) {
        const double volume = mesh.getElementVolume(element);
        for (int vertex = 0; vertex < mesh.getNumElementVertices(); vertex++) {
            vertex_volumes[mesh.getVertexIndex(element, vertex)] += factor * volume;
        }
    }
}

template <class MeshT>
double get_mass_impl(const MeshT& mesh) {
    double mass = 0.0;
    for (int region_index = 0; region_index < mesh.getNumRegions(); region_index++) {
        const MaterialRegion& region = mesh.getRegion(region_index);
        const double density         = material_density(mesh.getMaterial(region.getMaterialIndex()));
        std::set<int> set_elements;
        mesh.getSet(region.getSetIndex()).getElements(set_elements);

        for (std::set<int>::iterator iter = set_elements.begin(); iter != set_elements.end(); iter++) {
            const int element = *iter;
            mass += mesh.getElementVolume(element) * density;
        }
    }

    return mass;
}

template <class MeshT>
void get_inertia_parameters_impl(const MeshT& mesh, double& mass, Vec3d& center_of_mass, Mat3d& inertia_tensor) {
    mass = 0.0;
    center_of_mass.setZero();
    inertia_tensor.setZero();

    for (int element = 0; element < mesh.getNumElements(); element++) {
        const double density        = mesh.getElementDensity(element);
        const double element_volume = mesh.getElementVolume(element);
        const double element_mass   = element_volume * density;

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

template <class MeshT>
void get_mesh_geometric_parameters_impl(const MeshT& mesh, Vec3d& centroid, double* radius) {
    centroid = Vec3d(0, 0, 0);
    for (int i = 0; i < mesh.getNumVertices(); i++) {
        centroid += mesh.getVertex(i);
    }

    centroid /= mesh.getNumVertices();

    *radius = 0;
    for (int i = 0; i < mesh.getNumVertices(); i++) {
        const Vec3d vertex = mesh.getVertex(i);
        const double dist  = (vertex - centroid).norm();
        if (dist > *radius) {
            *radius = dist;
        }
    }
}

template <class MeshT>
Mesh::BoundingBox get_bounding_box_impl(const MeshT& mesh) {
    const auto vertices = mesh.getVertices();
    return Mesh::BoundingBox(std::vector<Vec3d>(vertices.begin(), vertices.end()));
}

}  // namespace detail

template <concepts::VolumetricMeshLike MeshT>
double get_volume(const MeshT& mesh) {
    return detail::get_volume_impl(mesh);
}

template <concepts::VolumetricMeshLike MeshT>
void get_vertex_volumes(const MeshT& mesh, double* vertex_volumes) {
    detail::get_vertex_volumes_impl(mesh, vertex_volumes);
}

template <concepts::VolumetricMeshLike MeshT>
double get_mass(const MeshT& mesh) {
    return detail::get_mass_impl(mesh);
}

template <concepts::VolumetricMeshLike MeshT>
void get_inertia_parameters(const MeshT& mesh, double& mass, Vec3d& center_of_mass, Mat3d& inertia_tensor) {
    detail::get_inertia_parameters_impl(mesh, mass, center_of_mass, inertia_tensor);
}

template <concepts::VolumetricMeshLike MeshT>
void get_mesh_geometric_parameters(const MeshT& mesh, Vec3d& centroid, double* radius) {
    detail::get_mesh_geometric_parameters_impl(mesh, centroid, radius);
}

template <concepts::VolumetricMeshLike MeshT>
Mesh::BoundingBox get_bounding_box(const MeshT& mesh) {
    return detail::get_bounding_box_impl(mesh);
}

double            get_volume(const ::pgo::VolumetricMeshes::VolumetricMesh& mesh);
void              get_vertex_volumes(const ::pgo::VolumetricMeshes::VolumetricMesh& mesh, double* vertex_volumes);
double            get_mass(const ::pgo::VolumetricMeshes::VolumetricMesh& mesh);
void              get_inertia_parameters(const ::pgo::VolumetricMeshes::VolumetricMesh& mesh, double& mass, Vec3d& center_of_mass,
                                         Mat3d& inertia_tensor);
void              get_mesh_geometric_parameters(const ::pgo::VolumetricMeshes::VolumetricMesh& mesh, Vec3d& centroid,
                                                double* radius);
Mesh::BoundingBox get_bounding_box(const ::pgo::VolumetricMeshes::VolumetricMesh& mesh);

}  // namespace pgo::VolumetricMeshes::internal::mesh_mass_properties
