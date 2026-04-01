#include "io/mesh_binary_writer.h"

#include "io/material_serde.h"

#include <fstream>
#include <stdexcept>
#include <type_traits>

namespace pgo::VolumetricMeshes::io::detail {
namespace {

template <typename T>
void write_exact(std::ostream& out, const T* data, size_t count) {
    out.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(sizeof(T) * count));
    if (!out) {
        throw std::runtime_error("Failed to write binary mesh data.");
    }
}

}  // namespace

void write_binary_mesh(const VolumetricMesh& mesh, std::ostream& out, unsigned int* bytes_written) {
    unsigned int total_bytes_written = 0;

    auto count_write = [&](const auto* data, size_t count) {
        using T = std::remove_pointer_t<decltype(data)>;
        write_exact(out, data, count);
        total_bytes_written += static_cast<unsigned int>(sizeof(T) * count);
    };

    const double version = 1.0;
    count_write(&version, 1);

    const int ele_type = static_cast<int>(mesh.getElementType());
    count_write(&ele_type, 1);
    const int num_vertices = mesh.getNumVertices();
    count_write(&num_vertices, 1);

    for (int vertex_index = 0; vertex_index < mesh.getNumVertices(); vertex_index++) {
        const Vec3d& vertex = mesh.getVertex(vertex_index);
        count_write(vertex.data(), 3);
    }

    const int num_elements = mesh.getNumElements();
    const int num_element_vertices = mesh.getNumElementVertices();
    count_write(&num_elements, 1);
    count_write(&num_element_vertices, 1);

    for (int element_index = 0; element_index < mesh.getNumElements(); element_index++) {
        const auto indices = mesh.getVertexIndices(element_index);
        count_write(indices.data(), static_cast<size_t>(mesh.getNumElementVertices()));
    }

    const int num_materials = mesh.getNumMaterials();
    count_write(&num_materials, 1);
    for (int material_index = 0; material_index < num_materials; material_index++) {
        total_bytes_written += write_binary_material(out, *mesh.getMaterial(material_index));
    }

    const int num_sets = mesh.getNumSets();
    count_write(&num_sets, 1);
    for (int set_index = 1; set_index < num_sets; set_index++) {
        const auto& set = mesh.getSet(set_index);
        const unsigned int length = static_cast<unsigned int>(set.getName().size());
        count_write(&length, 1);
        out.write(set.getName().c_str(), static_cast<std::streamsize>(length));
        if (!out) {
            throw std::runtime_error("Failed to write set name.");
        }
        total_bytes_written += length;

        const unsigned int cardinality = static_cast<unsigned int>(set.getNumElements());
        count_write(&cardinality, 1);
        std::vector<int> element_group(set.getElements().begin(), set.getElements().end());
        count_write(element_group.data(), element_group.size());
    }

    const int num_regions = mesh.getNumRegions();
    count_write(&num_regions, 1);
    for (int region_index = 0; region_index < mesh.getNumRegions(); region_index++) {
        const auto& region = mesh.getRegion(region_index);
        const int material_index = region.getMaterialIndex();
        const int set_index = region.getSetIndex();
        count_write(&material_index, 1);
        count_write(&set_index, 1);
    }

    if (bytes_written != nullptr) {
        *bytes_written = total_bytes_written;
    }
}

void write_binary_mesh(const VolumetricMesh& mesh, const std::filesystem::path& filename,
                       unsigned int* bytes_written) {
    std::ofstream out(filename, std::ios::binary);
    if (!out) {
        printf("Error: could not write to %s.\n", filename.string().c_str());
        throw std::runtime_error("Failed to open binary mesh file.");
    }

    write_binary_mesh(mesh, out, bytes_written);
}

}  // namespace pgo::VolumetricMeshes::io::detail
