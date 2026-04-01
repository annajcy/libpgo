#include "algorithms/cubic_mesh_refinement.h"

#include "cubicMesh.h"

#include "internal/mesh_mutation.h"

#include <vector>

namespace pgo::VolumetricMeshes::algorithms {

void subdivide_cubic_mesh(CubicMesh& mesh) {
    const int num_elements = mesh.getNumElements();
    const int num_new_elements = 8 * num_elements;
    std::vector<int> new_elements(num_new_elements * 8);

    const int parent_mask[8][3] = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
    };

    int mask[8][8][3];
    for (int el = 0; el < 8; ++el) {
        for (int vtx = 0; vtx < 8; ++vtx) {
            for (int dim = 0; dim < 3; ++dim) {
                mask[el][vtx][dim] = parent_mask[el][dim] + parent_mask[vtx][dim];
            }
        }
    }

    std::vector<Vec3d> new_vertices;
    for (int el = 0; el < num_elements; ++el) {
        const Vec3d& v0 = mesh.getVertex(el, 0);
        for (int child = 0; child < 8; ++child) {
            int child_vtx[8];
            for (int vtx = 0; vtx < 8; ++vtx) {
                const Vec3d pos =
                    v0 + 0.5 * mesh.cubeSize * Vec3d(mask[child][vtx][0], mask[child][vtx][1], mask[child][vtx][2]);

                int found = -1;
                for (int i = 0; i < static_cast<int>(new_vertices.size()); ++i) {
                    if ((pos - new_vertices[i]).norm() < 0.25 * mesh.cubeSize) {
                        found = i;
                        break;
                    }
                }

                if (found == -1) {
                    new_vertices.push_back(pos);
                    found = static_cast<int>(new_vertices.size()) - 1;
                }

                child_vtx[vtx] = found;
                new_elements[(8 * el + child) * 8 + vtx] = child_vtx[vtx];
            }
        }
    }

    mesh.cubeSize *= 0.5;
    mesh.SetInverseCubeSize();

    internal::MeshMutation::replace_geometry(
        mesh, internal::VolumetricMeshData(8, std::move(new_vertices), std::move(new_elements)));
    internal::MeshMutation::material_catalog(mesh).expand_elements(8);
}

}  // namespace pgo::VolumetricMeshes::algorithms
