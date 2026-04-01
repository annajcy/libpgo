#pragma once

#include "meshLinearAlgebra.h"

namespace pgo {
namespace VolumetricMeshes {

class CubicMesh;

namespace ops {

void compute_alpha_beta_gamma(const CubicMesh& mesh, int element, Vec3d pos, double* alpha, double* beta,
                              double* gamma);
bool contains_vertex(const CubicMesh& mesh, int element, Vec3d pos);
void compute_barycentric_weights(const CubicMesh& mesh, int element, const Vec3d& pos, double* weights);
void interpolate_gradient(const CubicMesh& mesh, int element, const double* U, int num_fields, Vec3d pos,
                          double* grad);

double element_volume(const CubicMesh& mesh, int element);
void   element_inertia_tensor(const CubicMesh& mesh, int element, Mat3d& inertia_tensor);
void   compute_element_mass_matrix(const CubicMesh& mesh, int element, double* mass_matrix);

int  num_element_edges(const CubicMesh& mesh);
void fill_element_edges(const CubicMesh& mesh, int element, int* edge_buffer);

}  // namespace ops
}  // namespace VolumetricMeshes
}  // namespace pgo
