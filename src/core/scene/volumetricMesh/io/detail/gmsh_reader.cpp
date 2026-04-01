#include "io/detail/gmsh_reader.h"

#include <gmsh.h>

#include <iostream>
#include <vector>

namespace pgo::VolumetricMeshes::io::detail {

LoadedMeshData read_gmsh_mesh(const std::filesystem::path& filename, int verbose) {
    gmsh::initialize();

    try {
        gmsh::open(filename.string());

        std::vector<std::size_t> node_tags;
        std::vector<double> node_coords;
        std::vector<double> node_params;
        gmsh::model::mesh::getNodes(node_tags, node_coords, node_params);

        if (verbose) {
            std::cout << "Loaded " << node_tags.size() << " nodes\n";
        }

        std::vector<Vec3d> vertices(node_tags.size());
        for (size_t i = 0; i < node_tags.size(); ++i) {
            vertices[i] = Vec3d(node_coords[3 * i + 0], node_coords[3 * i + 1], node_coords[3 * i + 2]);
        }

        std::vector<std::size_t> elem_tags;
        std::vector<std::size_t> elem_nodes;
        gmsh::model::mesh::getElementsByType(4, elem_tags, elem_nodes);

        if (verbose) {
            std::cout << "Read " << elem_tags.size() << " tetrahedra\n";
        }

        std::vector<int> elements(elem_tags.size() * 4);
        for (size_t i = 0; i < elem_tags.size(); ++i) {
            for (int j = 0; j < 4; ++j) {
                elements[i * 4 + j] = static_cast<int>(elem_nodes[4 * i + j] - 1);
            }
        }

        gmsh::finalize();

        internal::VolumetricMeshData geometry(4, std::move(vertices), std::move(elements));
        internal::MaterialCatalog material_catalog(geometry.num_elements(), VolumetricMesh::E_default,
                                                   VolumetricMesh::nu_default, VolumetricMesh::density_default);
        material_catalog.validate_against_num_elements(geometry.num_elements());
        return LoadedMeshData{VolumetricMesh::ElementType::Tet, std::move(geometry), std::move(material_catalog)};
    } catch (...) {
        gmsh::finalize();
        throw;
    }
}

}  // namespace pgo::VolumetricMeshes::io::detail
