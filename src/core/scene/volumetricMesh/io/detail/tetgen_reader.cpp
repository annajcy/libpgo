#include "io/detail/tetgen_reader.h"

#include "io/detail/veg_parser.h"

#include <cstdio>

namespace pgo::VolumetricMeshes::io::detail {

LoadedMeshData read_tetgen_mesh(const std::filesystem::path& filename, int verbose) {
    (void)verbose;
    char line_buffer[1024];
    VolumetricMeshParser parser;
    const std::string node_filename = filename.string() + ".node";
    const std::string ele_filename = filename.string() + ".ele";

    if (parser.open(node_filename.c_str()) != 0) {
        throw 2;
    }

    parser.getNextLine(line_buffer, 1);
    int dim = 0;
    int num_vertices = 0;
    sscanf(line_buffer, "%d %d", &num_vertices, &dim);
    if (dim != 3) {
        throw 3;
    }

    std::vector<Vec3d> vertices(static_cast<size_t>(num_vertices));
    for (int vertex_index = 0; vertex_index < num_vertices; vertex_index++) {
        parser.getNextLine(line_buffer, 1);
        int index = 0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        sscanf(line_buffer, "%d %lf %lf %lf", &index, &x, &y, &z);
        if (index != (vertex_index + 1)) {
            throw 3;
        }
        vertices[static_cast<size_t>(vertex_index)] = Vec3d(x, y, z);
    }

    parser.close();

    if (parser.open(ele_filename.c_str()) != 0) {
        throw 4;
    }

    parser.getNextLine(line_buffer, 1);
    int num_elements = 0;
    sscanf(line_buffer, "%d %d", &num_elements, &dim);
    if (dim != 4) {
        printf("Error: not a tet mesh file (%d vertices per tet encountered).\n", dim);
        throw 5;
    }

    std::vector<int> elements(static_cast<size_t>(4 * num_elements));
    for (int element_index = 0; element_index < num_elements; element_index++) {
        parser.getNextLine(line_buffer, 1);
        int index = 0;
        int v[4];
        sscanf(line_buffer, "%d %d %d %d %d", &index, &v[0], &v[1], &v[2], &v[3]);
        if (index != (element_index + 1)) {
            throw 6;
        }
        for (int vertex_slot = 0; vertex_slot < 4; vertex_slot++) {
            elements[static_cast<size_t>(element_index * 4 + vertex_slot)] = v[vertex_slot] - 1;
        }
    }

    parser.close();

    internal::VolumetricMeshData geometry(4, std::move(vertices), std::move(elements));
    internal::MaterialCatalog material_catalog(geometry.num_elements(), VolumetricMesh::E_default,
                                               VolumetricMesh::nu_default, VolumetricMesh::density_default);
    material_catalog.validate_against_num_elements(geometry.num_elements());
    return LoadedMeshData{ElementType::Tet, std::move(geometry), std::move(material_catalog)};
}

}  // namespace pgo::VolumetricMeshes::io::detail
