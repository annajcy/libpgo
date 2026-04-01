#include "io/mesh_binary_reader.h"

#include "io/material_serde.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace pgo::VolumetricMeshes::io::detail {
namespace {

using ElementType = VolumetricMesh::ElementType;

struct RawLoadedMeshData {
    ElementType element_type = VolumetricMesh::ElementType::Invalid;
    int num_element_vertices = 0;
    std::vector<Vec3d> vertices;
    std::vector<int> elements;
    std::vector<MaterialRecord> materials;
    std::vector<ElementSet> sets;
    std::vector<MaterialRegion> regions;
};

template <typename T>
void read_exact(std::istream& input, T* data, size_t count) {
    input.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(sizeof(T) * count));
    if (!input) {
        throw std::runtime_error("Failed to read binary mesh data.");
    }
}

ElementType element_type_from_int(int ele_type) {
    switch (ele_type) {
        case static_cast<int>(ElementType::Tet):
            return ElementType::Tet;
        case static_cast<int>(ElementType::Cubic):
            return ElementType::Cubic;
        default:
            return ElementType::Invalid;
    }
}

LoadedMeshData make_loaded_mesh_data(RawLoadedMeshData raw, int verbose) {
    internal::VolumetricMeshData geometry(raw.num_element_vertices, std::move(raw.vertices), std::move(raw.elements));
    internal::MaterialCatalog material_catalog(std::move(raw.materials), std::move(raw.sets), std::move(raw.regions),
                                               geometry.num_elements(), verbose);
    return LoadedMeshData{raw.element_type, std::move(geometry), std::move(material_catalog)};
}

}  // namespace

LoadedMeshData read_binary_mesh(std::istream& input) {
    RawLoadedMeshData data;

    double version = 0.0;
    read_exact(input, &version, 1);

    int ele_type = 0;
    read_exact(input, &ele_type, 1);
    data.element_type = element_type_from_int(ele_type);
    if (data.element_type == ElementType::Invalid) {
        printf("Error in io::load_binary_data: unknown mesh type %d in file stream\n", ele_type);
        throw 2;
    }

    int num_vertices = 0;
    read_exact(input, &num_vertices, 1);
    if (num_vertices < 0) {
        printf("Error in io::load_binary_data: incorrect number of vertices.\n");
        throw 3;
    }

    data.vertices.resize(static_cast<size_t>(num_vertices));
    read_exact(input, reinterpret_cast<double*>(data.vertices.data()), static_cast<size_t>(3 * num_vertices));

    int num_elements = 0;
    read_exact(input, &num_elements, 1);
    if (num_elements < 0) {
        printf("Error in io::load_binary_data: incorrect number of elements.\n");
        throw 4;
    }

    read_exact(input, &data.num_element_vertices, 1);
    if (data.num_element_vertices <= 0) {
        printf("Error in io::load_binary_data: incorrect number of vertices per element.\n");
        throw 5;
    }

    data.elements.resize(static_cast<size_t>(num_elements * data.num_element_vertices));
    read_exact(input, data.elements.data(), data.elements.size());

    int num_materials = 0;
    read_exact(input, &num_materials, 1);
    if (num_materials < 0) {
        printf("Error in io::load_binary_data: incorrect number of materials.\n");
        throw 6;
    }

    data.materials.resize(static_cast<size_t>(num_materials));
    for (int material_index = 0; material_index < num_materials; material_index++) {
        data.materials[static_cast<size_t>(material_index)] = read_binary_material(input);
    }

    int num_sets = 0;
    read_exact(input, &num_sets, 1);
    if (num_sets <= 0) {
        printf("Error in io::load_binary_data: incorrect number of sets.\n");
        throw 10;
    }

    data.sets.resize(static_cast<size_t>(num_sets));
    data.sets[0] = VolumetricMesh::generateAllElementsSet(num_elements);

    std::vector<int> int_temp_vec;
    for (int set_index = 1; set_index < num_sets; set_index++) {
        char set_name[4096];
        int length = 0;
        read_exact(input, &length, 1);
        read_exact(input, set_name, static_cast<size_t>(length));
        set_name[length] = '\0';
        data.sets[static_cast<size_t>(set_index)] = ElementSet(set_name);

        int cardinality = 0;
        read_exact(input, &cardinality, 1);
        int_temp_vec.resize(static_cast<size_t>(cardinality));
        read_exact(input, int_temp_vec.data(), int_temp_vec.size());
        for (int set_element_index = 0; set_element_index < cardinality; set_element_index++) {
            data.sets[static_cast<size_t>(set_index)].insert(
                int_temp_vec[static_cast<size_t>(set_element_index)]);
        }
    }

    int num_regions = 0;
    read_exact(input, &num_regions, 1);
    if (num_regions < 0) {
        printf("Error in io::load_binary_data: incorrect number of regions.\n");
        throw 11;
    }

    data.regions.resize(static_cast<size_t>(num_regions));
    for (int region_index = 0; region_index < num_regions; region_index++) {
        int material_index = 0;
        int set_index = 0;
        read_exact(input, &material_index, 1);
        read_exact(input, &set_index, 1);
        data.regions[static_cast<size_t>(region_index)] = MaterialRegion(material_index, set_index);
    }

    return make_loaded_mesh_data(std::move(data), 0);
}

LoadedMeshData read_binary_mesh(std::span<const std::byte> binary_data) {
    if (binary_data.empty()) {
        throw std::runtime_error("binary data buffer is empty");
    }

    std::string buffer(reinterpret_cast<const char*>(binary_data.data()), binary_data.size());
    std::istringstream input(buffer, std::ios::binary);
    return read_binary_mesh(input);
}

LoadedMeshData read_binary_mesh(const std::filesystem::path& filename) {
    std::ifstream input(filename, std::ios::binary);
    if (!input) {
        printf("Error in io::load_data: could not open file %s.\n", filename.string().c_str());
        throw 1;
    }
    return read_binary_mesh(input);
}

}  // namespace pgo::VolumetricMeshes::io::detail
