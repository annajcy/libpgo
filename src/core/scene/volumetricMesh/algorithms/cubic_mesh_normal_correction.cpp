#include "algorithms/cubic_mesh_normal_correction.h"

#include "cubicMesh.h"

#include "EigenSupport.h"

#include <cfloat>
#include <cassert>

namespace pgo::VolumetricMeshes::algorithms {

namespace ES = EigenSupport;

namespace {

Vec3d location_at(const double* interpolation_locations, int index) {
    return Vec3d(interpolation_locations[3 * index + 0], interpolation_locations[3 * index + 1],
                 interpolation_locations[3 * index + 2]);
}

double minimum_distance_to_element_vertices(const CubicMesh& mesh, int element, const Vec3d& pos) {
    double min_distance = DBL_MAX;
    for (int i = 0; i < mesh.getNumElementVertices(); ++i) {
        const Vec3d& vpos = mesh.getVertex(element, i);
        const double distance = (vpos - pos).norm();
        if (distance < min_distance) {
            min_distance = distance;
        }
    }
    return min_distance;
}

}  // namespace

int cubic_mesh_normal_correction(const CubicMesh& mesh, double* vertex_data, int num_locations, int r,
                                 double* interpolation_locations, double* static_normals,
                                 double* normal_correction, double distance_threshold) {
    assert(mesh.getNumElementVertices() == 8);
    const int num_vertices = mesh.getNumVertices();
    const double inv_cube_size = (mesh.getCubeSize() > 0.0) ? 1.0 / mesh.getCubeSize() : 0.0;

    int num_external_vertices = 0;
    for (int i = 0; i < num_locations; ++i) {
        if (i % 100 == 0) {
            printf("%d ", i);
            fflush(nullptr);
        }

        const Vec3d pos = location_at(interpolation_locations, i);
        const int element = mesh.getClosestElement(pos);

        if (!mesh.containsVertex(element, pos)) {
            num_external_vertices++;
        }

        if (distance_threshold > 0.0) {
            const double min_distance = minimum_distance_to_element_vertices(mesh, element, pos);
            if (min_distance > distance_threshold) {
                for (int j = 0; j < r; ++j) {
                    normal_correction[ES::ELT(3 * num_locations, 3 * i + 0, j)] = 0;
                    normal_correction[ES::ELT(3 * num_locations, 3 * i + 1, j)] = 0;
                    normal_correction[ES::ELT(3 * num_locations, 3 * i + 2, j)] = 0;
                }
                continue;
            }
        }

        const Vec3d w = pos - mesh.getVertex(element, 0);
        const double alpha = w[0] * inv_cube_size;
        const double beta = w[1] * inv_cube_size;
        const double gamma = w[2] * inv_cube_size;

        const int v000 = mesh.getVertexIndex(element, 0);
        const int v100 = mesh.getVertexIndex(element, 1);
        const int v110 = mesh.getVertexIndex(element, 2);
        const int v010 = mesh.getVertexIndex(element, 3);
        const int v001 = mesh.getVertexIndex(element, 4);
        const int v101 = mesh.getVertexIndex(element, 5);
        const int v111 = mesh.getVertexIndex(element, 6);
        const int v011 = mesh.getVertexIndex(element, 7);

        const Vec3d gradf000(inv_cube_size * -(1 - beta) * (1 - gamma), inv_cube_size * -(1 - alpha) * (1 - gamma),
                             inv_cube_size * -(1 - alpha) * (1 - beta));
        const Vec3d gradf100(inv_cube_size * (1 - beta) * (1 - gamma), inv_cube_size * -alpha * (1 - gamma),
                             inv_cube_size * -alpha * (1 - beta));
        const Vec3d gradf110(inv_cube_size * beta * (1 - gamma), inv_cube_size * alpha * (1 - gamma),
                             inv_cube_size * -alpha * beta);
        const Vec3d gradf010(inv_cube_size * -beta * (1 - gamma), inv_cube_size * (1 - alpha) * (1 - gamma),
                             inv_cube_size * (1 - alpha) * -beta);
        const Vec3d gradf001(inv_cube_size * -(1 - beta) * gamma, inv_cube_size * -(1 - alpha) * gamma,
                             inv_cube_size * (1 - alpha) * (1 - beta));
        const Vec3d gradf101(inv_cube_size * (1 - beta) * gamma, inv_cube_size * -alpha * gamma,
                             inv_cube_size * alpha * (1 - beta));
        const Vec3d gradf111(inv_cube_size * beta * gamma, inv_cube_size * alpha * gamma, inv_cube_size * alpha * beta);
        const Vec3d gradf011(inv_cube_size * -beta * gamma, inv_cube_size * (1 - alpha) * gamma,
                             inv_cube_size * (1 - alpha) * beta);

        const Vec3d normal(static_normals[3 * i + 0], static_normals[3 * i + 1], static_normals[3 * i + 2]);

        for (int j = 0; j < r; ++j) {
            const Vec3d u000 = Vec3d(vertex_data[ES::ELT(3 * num_vertices, 3 * v000 + 0, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v000 + 1, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v000 + 2, j)]);
            const Vec3d u100 = Vec3d(vertex_data[ES::ELT(3 * num_vertices, 3 * v100 + 0, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v100 + 1, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v100 + 2, j)]);
            const Vec3d u110 = Vec3d(vertex_data[ES::ELT(3 * num_vertices, 3 * v110 + 0, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v110 + 1, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v110 + 2, j)]);
            const Vec3d u010 = Vec3d(vertex_data[ES::ELT(3 * num_vertices, 3 * v010 + 0, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v010 + 1, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v010 + 2, j)]);
            const Vec3d u001 = Vec3d(vertex_data[ES::ELT(3 * num_vertices, 3 * v001 + 0, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v001 + 1, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v001 + 2, j)]);
            const Vec3d u101 = Vec3d(vertex_data[ES::ELT(3 * num_vertices, 3 * v101 + 0, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v101 + 1, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v101 + 2, j)]);
            const Vec3d u111 = Vec3d(vertex_data[ES::ELT(3 * num_vertices, 3 * v111 + 0, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v111 + 1, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v111 + 2, j)]);
            const Vec3d u011 = Vec3d(vertex_data[ES::ELT(3 * num_vertices, 3 * v011 + 0, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v011 + 1, j)],
                                     vertex_data[ES::ELT(3 * num_vertices, 3 * v011 + 2, j)]);

            Vec3d coef(0, 0, 0);
            coef += gradf000.dot(normal) * u000;
            coef += gradf100.dot(normal) * u100;
            coef += gradf110.dot(normal) * u110;
            coef += gradf010.dot(normal) * u010;
            coef += gradf001.dot(normal) * u001;
            coef += gradf101.dot(normal) * u101;
            coef += gradf111.dot(normal) * u111;
            coef += gradf011.dot(normal) * u011;

            normal_correction[ES::ELT(3 * num_locations, 3 * i + 0, j)] = coef[0];
            normal_correction[ES::ELT(3 * num_locations, 3 * i + 1, j)] = coef[1];
            normal_correction[ES::ELT(3 * num_locations, 3 * i + 2, j)] = coef[2];
        }
    }

    printf("\n");
    return num_external_vertices;
}

}  // namespace pgo::VolumetricMeshes::algorithms
