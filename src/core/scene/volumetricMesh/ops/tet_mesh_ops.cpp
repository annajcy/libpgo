#include "ops/tet_mesh_ops.h"

#include "tetMesh.h"

#include "geometryQuery.h"
#include "predicates.h"
#include "tetMeshGeo.h"
#include "EigenSupport.h"

#include <cfloat>
#include <cmath>

namespace pgo {
namespace VolumetricMeshes {
namespace ops {

double tet_determinant(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d) {
    return Mesh::getTetDeterminant(a, b, c, d);
}

double signed_tet_volume(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d) {
    return tet_determinant(a, b, c, d) / 6.0;
}

double tet_volume(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d) {
    return std::abs(tet_determinant(a, b, c, d)) / 6.0;
}

double element_volume(const TetMesh& mesh, int element) {
    return tet_volume(mesh.getVertex(element, 0), mesh.getVertex(element, 1), mesh.getVertex(element, 2),
                      mesh.getVertex(element, 3));
}

void element_inertia_tensor(const TetMesh& mesh, int element, Mat3d& inertia_tensor) {
    Vec3d a = mesh.getVertex(element, 0);
    Vec3d b = mesh.getVertex(element, 1);
    Vec3d c = mesh.getVertex(element, 2);
    Vec3d d = mesh.getVertex(element, 3);

    const Vec3d center = mesh.getElementCenter(element);
    a -= center;
    b -= center;
    c -= center;
    d -= center;

    const double absdet_j = std::abs(tet_determinant(a, b, c, d));

    const double x1 = a[0], x2 = b[0], x3 = c[0], x4 = d[0];
    const double y1 = a[1], y2 = b[1], y3 = c[1], y4 = d[1];
    const double z1 = a[2], z2 = b[2], z3 = c[2], z4 = d[2];

    const double A =
        absdet_j * (y1 * y1 + y1 * y2 + y2 * y2 + y1 * y3 + y2 * y3 + y3 * y3 + y1 * y4 + y2 * y4 + y3 * y4 +
                    y4 * y4 + z1 * z1 + z1 * z2 + z2 * z2 + z1 * z3 + z2 * z3 + z3 * z3 + z1 * z4 + z2 * z4 +
                    z3 * z4 + z4 * z4) /
        60.0;

    const double B =
        absdet_j * (x1 * x1 + x1 * x2 + x2 * x2 + x1 * x3 + x2 * x3 + x3 * x3 + x1 * x4 + x2 * x4 + x3 * x4 +
                    x4 * x4 + z1 * z1 + z1 * z2 + z2 * z2 + z1 * z3 + z2 * z3 + z3 * z3 + z1 * z4 + z2 * z4 +
                    z3 * z4 + z4 * z4) /
        60.0;

    const double C =
        absdet_j * (x1 * x1 + x1 * x2 + x2 * x2 + x1 * x3 + x2 * x3 + x3 * x3 + x1 * x4 + x2 * x4 + x3 * x4 +
                    x4 * x4 + y1 * y1 + y1 * y2 + y2 * y2 + y1 * y3 + y2 * y3 + y3 * y3 + y1 * y4 + y2 * y4 +
                    y3 * y4 + y4 * y4) /
        60.0;

    const double Ap =
        absdet_j *
        (2 * y1 * z1 + y2 * z1 + y3 * z1 + y4 * z1 + y1 * z2 + 2 * y2 * z2 + y3 * z2 + y4 * z2 + y1 * z3 +
         y2 * z3 + 2 * y3 * z3 + y4 * z3 + y1 * z4 + y2 * z4 + y3 * z4 + 2 * y4 * z4) /
        120.0;

    const double Bp =
        absdet_j *
        (2 * x1 * z1 + x2 * z1 + x3 * z1 + x4 * z1 + x1 * z2 + 2 * x2 * z2 + x3 * z2 + x4 * z2 + x1 * z3 +
         x2 * z3 + 2 * x3 * z3 + x4 * z3 + x1 * z4 + x2 * z4 + x3 * z4 + 2 * x4 * z4) /
        120.0;

    const double Cp =
        absdet_j *
        (2 * x1 * y1 + x2 * y1 + x3 * y1 + x4 * y1 + x1 * y2 + 2 * x2 * y2 + x3 * y2 + x4 * y2 + x1 * y3 +
         x2 * y3 + 2 * x3 * y3 + x4 * y3 + x1 * y4 + x2 * y4 + x3 * y4 + 2 * x4 * y4) /
        120.0;

    inertia_tensor = asMat3d(A, -Bp, -Cp, -Bp, B, -Ap, -Cp, -Ap, C);
}

void compute_element_mass_matrix(const TetMesh& mesh, int element, double* mass_matrix) {
    const double mtx[16] = {2, 1, 1, 1, 1, 2, 1, 1, 1, 1, 2, 1, 1, 1, 1, 2};
    const double factor = mesh.getElementDensity(element) * element_volume(mesh, element) / 20.0;

    for (int i = 0; i < 16; ++i) {
        mass_matrix[i] = factor * mtx[i];
    }
}

bool contains_vertex(const TetMesh& mesh, int element, Vec3d pos) {
    return Mesh::pointInTet(pos.data(), mesh.getVertex(element, 0).data(), mesh.getVertex(element, 1).data(),
                            mesh.getVertex(element, 2).data(), mesh.getVertex(element, 3).data());
}

void compute_barycentric_weights(std::span<const Vec3d, 4> tet_vertex_pos, const Vec3d& pos, double weights[4]) {
    compute_barycentric_weights(tet_vertex_pos[0], tet_vertex_pos[1], tet_vertex_pos[2], tet_vertex_pos[3], pos,
                                weights);
}

void compute_barycentric_weights(const Vec3d& tet_vtx_pos0, const Vec3d& tet_vtx_pos1, const Vec3d& tet_vtx_pos2,
                                 const Vec3d& tet_vtx_pos3, const Vec3d& pos, double weights[4]) {
    Mesh::getTetBarycentricWeights(pos, tet_vtx_pos0, tet_vtx_pos1, tet_vtx_pos2, tet_vtx_pos3, weights);
}

void compute_barycentric_weights(const TetMesh& mesh, int element, const Vec3d& pos, double* weights) {
    Vec3d tet_vertex_pos[4];
    for (int i = 0; i < 4; ++i) {
        tet_vertex_pos[i] = mesh.getVertex(element, i);
    }

    compute_barycentric_weights(std::span<const Vec3d, 4>(tet_vertex_pos, 4), pos, weights);
}

void compute_gradient(const TetMesh& mesh, int element, const double* U, int num_fields, double* grad) {
    Vec3d vtx[4];
    for (int i = 0; i < 4; ++i) {
        vtx[i] = mesh.getVertex(element, i);
    }

    const Mat3d M = asMat3d(vtx[1] - vtx[0], vtx[2] - vtx[0], vtx[3] - vtx[0]);
    const Mat3d M_inv = M.fullPivLu().inverse();

    for (int field = 0; field < num_fields; ++field) {
        const double* u[4];
        for (int i = 0; i < 4; ++i) {
            u[i] = &U[3 * mesh.getNumVertices() * field + 3 * mesh.getVertexIndex(element, i)];
        }

        Vec3d rows[3];
        for (int i = 0; i < 3; ++i) {
            rows[i] = asVec3d(u[i + 1]) - asVec3d(u[0]);
        }

        const Mat3d rhs = asMat3d(rows[0], rows[1], rows[2]);
        const Mat3d grad_m = (M_inv * rhs).transpose();
        (EigenSupport::Mp<Mat3d>)(&grad[9 * field]) = grad_m;
    }
}

int num_element_edges(const TetMesh&) {
    return 6;
}

void fill_element_edges(const TetMesh& mesh, int element, int* edge_buffer) {
    int v[4];
    for (int i = 0; i < 4; ++i) {
        v[i] = mesh.getVertexIndex(element, i);
    }

    const int edge_mask[6][2] = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
    for (int edge = 0; edge < 6; ++edge) {
        edge_buffer[2 * edge + 0] = v[edge_mask[edge][0]];
        edge_buffer[2 * edge + 1] = v[edge_mask[edge][1]];
    }
}

int closest_element(const TetMesh& mesh, const Vec3d& pos) {
    double closest_dist = DBL_MAX;
    int    closest_element_index = 0;

    for (int element = 0; element < mesh.getNumElements(); ++element) {
        const double dist = Mesh::getSquaredDistanceToTet(pos, mesh.getVertex(element, 0), mesh.getVertex(element, 1),
                                                          mesh.getVertex(element, 2), mesh.getVertex(element, 3));
        if (dist < closest_dist) {
            closest_dist = dist;
            closest_element_index = element;
        }
    }

    return closest_element_index;
}

}  // namespace ops
}  // namespace VolumetricMeshes
}  // namespace pgo
