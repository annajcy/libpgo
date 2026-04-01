#include "io/mesh_ascii_writer.h"

#include "io/material_serde.h"

#include <cstdio>
#include <stdexcept>

namespace pgo::VolumetricMeshes::io::detail {

void write_ascii_mesh(const VolumetricMesh& mesh, const std::filesystem::path& filename) {
    FILE* output = fopen(filename.string().c_str(), "w");
    if (!output) {
        printf("Error: could not write to %s.\n", filename.string().c_str());
        throw std::runtime_error("Failed to open ascii mesh file.");
    }

    fprintf(output, "# Vega mesh file.\n");
    fprintf(output, "# %d vertices, %d elements\n\n", mesh.getNumVertices(), mesh.getNumElements());

    fprintf(output, "*VERTICES\n");
    fprintf(output, "%d 3 0 0\n", mesh.getNumVertices());
    for (int vertex_index = 0; vertex_index < mesh.getNumVertices(); vertex_index++) {
        const Vec3d& vertex = mesh.getVertex(vertex_index);
        fprintf(output, "%d %.15G %.15G %.15G\n", vertex_index + 1, vertex[0], vertex[1], vertex[2]);
    }
    fprintf(output, "\n");

    fprintf(output, "*ELEMENTS\n");
    fprintf(output, "%s\n", mesh.getElementType() == VolumetricMesh::ElementType::Tet ? "TET" : "CUBIC");
    fprintf(output, "%d %d 0\n", mesh.getNumElements(), mesh.getNumElementVertices());
    for (int element_index = 0; element_index < mesh.getNumElements(); element_index++) {
        fprintf(output, "%d ", element_index + 1);
        for (int vertex_slot = 0; vertex_slot < mesh.getNumElementVertices(); vertex_slot++) {
            fprintf(output, "%d", mesh.getVertexIndex(element_index, vertex_slot) + 1);
            if (vertex_slot != mesh.getNumElementVertices() - 1) {
                fprintf(output, " ");
            }
        }
        fprintf(output, "\n");
    }
    fprintf(output, "\n");

    for (int material_index = 0; material_index < mesh.getNumMaterials(); material_index++) {
        write_ascii_material(output, mesh.getMaterial(material_index));
    }

    for (int set_index = 1; set_index < mesh.getNumSets(); set_index++) {
        const auto& set = mesh.getSet(set_index);
        fprintf(output, "*SET %s\n", set.getName().c_str());
        int count = 0;
        for (int element : set.getElements()) {
            fprintf(output, "%d, ", element + 1);
            count++;
            if (count == 8) {
                fprintf(output, "\n");
                count = 0;
            }
        }
        if (count != 0) {
            fprintf(output, "\n");
        }
        fprintf(output, "\n");
    }

    for (int region_index = 0; region_index < mesh.getNumRegions(); region_index++) {
        const auto& region = mesh.getRegion(region_index);
        fprintf(output, "*REGION\n");
        fprintf(output, "%s, %s\n", mesh.getSet(region.getSetIndex()).getName().c_str(),
                mesh.getMaterial(region.getMaterialIndex()).name.c_str());
        fprintf(output, "\n");
    }

    fclose(output);
}

}  // namespace pgo::VolumetricMeshes::io::detail
