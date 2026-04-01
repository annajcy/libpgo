#pragma once

#include "meshLinearAlgebra.h"

#include <span>

namespace pgo {
namespace VolumetricMeshes {

class TetMesh;

namespace ops {

double tet_determinant(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d);
double signed_tet_volume(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d);
double tet_volume(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d);

double element_volume(const TetMesh& mesh, int element);
void   element_inertia_tensor(const TetMesh& mesh, int element, Mat3d& inertia_tensor);
void   compute_element_mass_matrix(const TetMesh& mesh, int element, double* mass_matrix);

bool contains_vertex(const TetMesh& mesh, int element, Vec3d pos);

void compute_barycentric_weights(std::span<const Vec3d, 4> tet_vertex_pos, const Vec3d& pos, double weights[4]);
void compute_barycentric_weights(const Vec3d& tet_vtx_pos0, const Vec3d& tet_vtx_pos1, const Vec3d& tet_vtx_pos2,
                                 const Vec3d& tet_vtx_pos3, const Vec3d& pos, double weights[4]);
void compute_barycentric_weights(const TetMesh& mesh, int element, const Vec3d& pos, double* weights);

void compute_gradient(const TetMesh& mesh, int element, const double* U, int num_fields, double* grad);

int  num_element_edges(const TetMesh& mesh);
void fill_element_edges(const TetMesh& mesh, int element, int* edge_buffer);

int closest_element(const TetMesh& mesh, const Vec3d& pos);

}  // namespace ops
}  // namespace VolumetricMeshes
}  // namespace pgo
