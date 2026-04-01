#include "io/mesh_load.h"

#include "io/mesh_ascii_reader.h"
#include "io/mesh_binary_reader.h"
#include "io/mesh_format_detector.h"
#include "io/mesh_loaders.h"

#include "cubicMesh.h"
#include "tetMesh.h"

namespace pgo::VolumetricMeshes::io {
namespace {

using LoadedMeshData = detail::LoadedMeshData;
using ElementType = VolumetricMesh::ElementType;
using FileFormatType = VolumetricMesh::FileFormatType;

constexpr ElementType TET = ElementType::Tet;
constexpr ElementType CUBIC = ElementType::Cubic;
constexpr FileFormatType ASCII = FileFormatType::Ascii;
constexpr FileFormatType BINARY = FileFormatType::Binary;
constexpr FileFormatType BY_EXT = FileFormatType::ByExtension;
constexpr FileFormatType UNKNOWN = FileFormatType::Unknown;

int vertices_per_element(ElementType elementType) {
    switch (elementType) {
        case TET:
            return 4;
        case CUBIC:
            return 8;
        default:
            return 0;
    }
}

void validate_loaded_data(const LoadedMeshData& data, ElementType expectedType) {
    if (data.element_type != expectedType) {
        printf("Error: mesh is not a %s mesh.\n", expectedType == TET ? "tet" : "cubic");
        throw 11;
    }
    const int expectedVerticesPerElement = vertices_per_element(expectedType);
    if (data.geometry.num_element_vertices() != expectedVerticesPerElement) {
        printf("Error: mesh data has %d vertices per element; expected %d.\n", data.geometry.num_element_vertices(),
               expectedVerticesPerElement);
        throw 12;
    }
    data.material_catalog.validate_against_num_elements(data.geometry.num_elements());
}

LoadedMeshData load_data(const std::filesystem::path& filename, FileFormatType fileFormat, int verbose) {
    if (verbose) {
        printf("Opening file %s.\n", filename.string().c_str());
        fflush(nullptr);
    }

    if (fileFormat == BY_EXT) {
        fileFormat = detail::detect_file_format_by_ext(filename);
        if (fileFormat == UNKNOWN)
            fileFormat = ASCII;
    }

    switch (fileFormat) {
        case ASCII:
            return detail::read_ascii_mesh(filename, verbose);
        case BINARY:
            return detail::read_binary_mesh(filename);
        default:
            printf("Error in io::load_data: file format is unknown.\n");
            throw 1;
    }
}

template <typename Mesh>
std::unique_ptr<Mesh> load_impl(const std::filesystem::path& filename, VolumetricMesh::FileFormatType fileFormat,
                                int verbose) {
    return std::make_unique<Mesh>(filename, fileFormat, verbose);
}

}  // namespace

namespace detail {

LoadedMeshData load_tet_data(const std::filesystem::path& filename, VolumetricMesh::FileFormatType file_format,
                             int verbose) {
    LoadedMeshData data = load_data(filename, file_format, verbose);
    validate_loaded_data(data, TET);
    return data;
}

LoadedMeshData load_tet_data(std::span<const std::byte> binary_data) {
    LoadedMeshData data = detail::read_binary_mesh(binary_data);
    validate_loaded_data(data, TET);
    return data;
}

LoadedMeshData load_cubic_data(const std::filesystem::path& filename, VolumetricMesh::FileFormatType file_format,
                               int verbose) {
    LoadedMeshData data = load_data(filename, file_format, verbose);
    validate_loaded_data(data, CUBIC);
    return data;
}

LoadedMeshData load_cubic_data(std::span<const std::byte> binary_data) {
    LoadedMeshData data = detail::read_binary_mesh(binary_data);
    validate_loaded_data(data, CUBIC);
    return data;
}

}  // namespace detail

std::unique_ptr<TetMesh> load_tet(const std::filesystem::path& filename, VolumetricMesh::FileFormatType fileFormat,
                                  int verbose) {
    return load_impl<TetMesh>(filename, fileFormat, verbose);
}

std::unique_ptr<CubicMesh> load_cubic(const std::filesystem::path& filename,
                                      VolumetricMesh::FileFormatType fileFormat, int verbose) {
    return load_impl<CubicMesh>(filename, fileFormat, verbose);
}

}  // namespace pgo::VolumetricMeshes::io
