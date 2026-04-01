#include "ops/cubic_mesh_ops.h"

#include "cubicMesh.h"

#include "EigenSupport.h"

namespace pgo {
namespace VolumetricMeshes {

namespace ES = EigenSupport;

namespace ops {

namespace {
double inverse_cube_size(const CubicMesh& mesh) {
    const double cube_size = mesh.getCubeSize();
    return (cube_size > 0.0) ? 1.0 / cube_size : 0.0;
}

void compute_shape_gradients(const CubicMesh& mesh, double alpha, double beta, double gamma, Vec3d gradients[8]) {
    const double inv_cube_size = inverse_cube_size(mesh);

    gradients[0] = Vec3d(inv_cube_size * -(1 - beta) * (1 - gamma), inv_cube_size * -(1 - alpha) * (1 - gamma),
                         inv_cube_size * -(1 - alpha) * (1 - beta));
    gradients[1] = Vec3d(inv_cube_size * (1 - beta) * (1 - gamma), inv_cube_size * -alpha * (1 - gamma),
                         inv_cube_size * -alpha * (1 - beta));
    gradients[2] = Vec3d(inv_cube_size * beta * (1 - gamma), inv_cube_size * alpha * (1 - gamma),
                         inv_cube_size * -alpha * beta);
    gradients[3] = Vec3d(inv_cube_size * -beta * (1 - gamma), inv_cube_size * (1 - alpha) * (1 - gamma),
                         inv_cube_size * (1 - alpha) * -beta);
    gradients[4] = Vec3d(inv_cube_size * -(1 - beta) * gamma, inv_cube_size * -(1 - alpha) * gamma,
                         inv_cube_size * (1 - alpha) * (1 - beta));
    gradients[5] = Vec3d(inv_cube_size * (1 - beta) * gamma, inv_cube_size * -alpha * gamma,
                         inv_cube_size * alpha * (1 - beta));
    gradients[6] = Vec3d(inv_cube_size * beta * gamma, inv_cube_size * alpha * gamma, inv_cube_size * alpha * beta);
    gradients[7] = Vec3d(inv_cube_size * -beta * gamma, inv_cube_size * (1 - alpha) * gamma,
                         inv_cube_size * (1 - alpha) * beta);
}
}  // namespace

void compute_alpha_beta_gamma(const CubicMesh& mesh, int element, Vec3d pos, double* alpha, double* beta,
                              double* gamma) {
    if (mesh.parallelepipedMode) {
        const Vec3d& v0 = mesh.getVertex(element, 0);
        const Vec3d& v1 = mesh.getVertex(element, 1);
        const Vec3d& v3 = mesh.getVertex(element, 3);
        const Vec3d& v4 = mesh.getVertex(element, 4);

        const Vec3d axis0 = v1 - v0;
        const Vec3d axis1 = v3 - v0;
        const Vec3d axis2 = v4 - v0;
        const Vec3d p = pos - v0;

        Mat3d A;
        A.col(0) = axis0;
        A.col(1) = axis1;
        A.col(2) = axis2;

        const Vec3d v = A.fullPivHouseholderQr().inverse() * p;
        *alpha = v[0];
        *beta = v[1];
        *gamma = v[2];
        return;
    }

    const Vec3d& v0 = mesh.getVertex(element, 0);
    const Vec3d p = pos - v0;
    const double inv_cube_size = inverse_cube_size(mesh);
    *alpha = p[0] * inv_cube_size;
    *beta = p[1] * inv_cube_size;
    *gamma = p[2] * inv_cube_size;
}

bool contains_vertex(const CubicMesh& mesh, int element, Vec3d pos) {
    double alpha = 0.0;
    double beta = 0.0;
    double gamma = 0.0;
    compute_alpha_beta_gamma(mesh, element, pos, &alpha, &beta, &gamma);
    return (0 <= alpha) && (alpha <= 1) && (0 <= beta) && (beta <= 1) && (0 <= gamma) && (gamma <= 1);
}

void compute_barycentric_weights(const CubicMesh& mesh, int element, const Vec3d& pos, double* weights) {
    double alpha = 0.0;
    double beta = 0.0;
    double gamma = 0.0;
    compute_alpha_beta_gamma(mesh, element, pos, &alpha, &beta, &gamma);

    weights[0] = (1 - alpha) * (1 - beta) * (1 - gamma);
    weights[1] = alpha * (1 - beta) * (1 - gamma);
    weights[2] = alpha * beta * (1 - gamma);
    weights[3] = (1 - alpha) * beta * (1 - gamma);
    weights[4] = (1 - alpha) * (1 - beta) * gamma;
    weights[5] = alpha * (1 - beta) * gamma;
    weights[6] = alpha * beta * gamma;
    weights[7] = (1 - alpha) * beta * gamma;
}

void interpolate_gradient(const CubicMesh& mesh, int element, const double* U, int num_fields, Vec3d pos,
                          double* grad) {
    double alpha = 0.0;
    double beta = 0.0;
    double gamma = 0.0;
    compute_alpha_beta_gamma(mesh, element, pos, &alpha, &beta, &gamma);

    Vec3d gradients[8];
    compute_shape_gradients(mesh, alpha, beta, gamma, gradients);

    const int num_vertices = mesh.getNumVertices();
    const int vertex_indices[8] = {mesh.getVertexIndex(element, 0), mesh.getVertexIndex(element, 1),
                                   mesh.getVertexIndex(element, 2), mesh.getVertexIndex(element, 3),
                                   mesh.getVertexIndex(element, 4), mesh.getVertexIndex(element, 5),
                                   mesh.getVertexIndex(element, 6), mesh.getVertexIndex(element, 7)};

    for (int field = 0; field < num_fields; ++field) {
        Mat3d F_mode = asMat3d(0.0);
        for (int i = 0; i < 8; ++i) {
            const int vertex = vertex_indices[i];
            const Vec3d u = Vec3d(U[ES::ELT(3 * num_vertices, 3 * vertex + 0, field)],
                                  U[ES::ELT(3 * num_vertices, 3 * vertex + 1, field)],
                                  U[ES::ELT(3 * num_vertices, 3 * vertex + 2, field)]);
            F_mode += EigenSupport::tensorProduct(u, gradients[i]);
        }

        for (int ii = 0; ii < 3; ++ii) {
            for (int jj = 0; jj < 3; ++jj) {
                grad[ES::ELT(9, 3 * ii + jj, field)] = F_mode(ii, jj);
            }
        }
    }
}

double element_volume(const CubicMesh& mesh, int) {
    const double cube_size = mesh.getCubeSize();
    return cube_size * cube_size * cube_size;
}

void element_inertia_tensor(const CubicMesh& mesh, int, Mat3d& inertia_tensor) {
    const double cube_size = mesh.getCubeSize();
    const double cube_size5 = cube_size * cube_size * cube_size * cube_size * cube_size;
    inertia_tensor = cube_size5 * asMat3d(1.0 / 6, 0, 0, 0, 1.0 / 6, 0, 0, 0, 1.0 / 6);
}

void compute_element_mass_matrix(const CubicMesh& mesh, int element, double* mass_matrix) {
    const double single_mass_matrix[64] = {
        0.0370370335876942,  0.0185185167938471,  0.00925925839692354, 0.0185185167938471,  0.0185185167938471,
        0.00925925839692354, 0.00462962919846177, 0.00925925839692354, 0.0185185167938471,  0.0370370335876942,
        0.0185185167938471,  0.00925925839692354, 0.00925925839692354, 0.0185185167938471,  0.00925925839692354,
        0.00462962919846177, 0.00925925839692354, 0.0185185167938471,  0.0370370335876942,  0.0185185167938471,
        0.00462962919846177, 0.00925925839692354, 0.0185185167938471,  0.00925925839692354, 0.0185185167938471,
        0.00925925839692354, 0.0185185167938471,  0.0370370335876942,  0.00925925839692354, 0.00462962919846177,
        0.00925925839692354, 0.0185185167938471,  0.0185185167938471,  0.00925925839692354, 0.00462962919846177,
        0.00925925839692354, 0.0370370335876942,  0.0185185167938471,  0.00925925839692354, 0.0185185167938471,
        0.00925925839692354, 0.0185185167938471,  0.00925925839692354, 0.00462962919846177, 0.0185185167938471,
        0.0370370335876942,  0.0185185167938471,  0.00925925839692354, 0.00462962919846177, 0.00925925839692354,
        0.0185185167938471,  0.00925925839692354, 0.00925925839692354, 0.0185185167938471,  0.0370370335876942,
        0.0185185167938471,  0.00925925839692354, 0.00462962919846177, 0.00925925839692354, 0.0185185167938471,
        0.0185185167938471,  0.00925925839692354, 0.0185185167938471,  0.0370370335876942};

    const double voxel_spacing = mesh.getCubeSize();
    const double factor = mesh.getElementDensity(element) * voxel_spacing * voxel_spacing * voxel_spacing;
    for (int i = 0; i < 64; ++i) {
        mass_matrix[i] = factor * single_mass_matrix[i];
    }
}

int num_element_edges(const CubicMesh&) {
    return 12;
}

void fill_element_edges(const CubicMesh& mesh, int element, int* edge_buffer) {
    int v[8];
    for (int i = 0; i < 8; ++i) {
        v[i] = mesh.getVertexIndex(element, i);
    }

    const int edge_mask[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                                  {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

    for (int edge = 0; edge < 12; ++edge) {
        edge_buffer[2 * edge + 0] = v[edge_mask[edge][0]];
        edge_buffer[2 * edge + 1] = v[edge_mask[edge][1]];
    }
}

}  // namespace ops
}  // namespace VolumetricMeshes
}  // namespace pgo
