#include "volumetricMeshIO.h"

#include "io/mesh_ascii_reader.h"
#include "io/mesh_ascii_writer.h"
#include "io/material_serde.h"
#include "io/mesh_format_detector.h"
#include "io/mesh_io_types.h"

#include "cubicMesh.h"
#include "tetMesh.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace pgo::VolumetricMeshes {
namespace io {
namespace {

using LoadedMeshData = detail::LoadedMeshData;
using ElementType = VolumetricMesh::ElementType;
using FileFormatType = VolumetricMesh::FileFormatType;

struct RawLoadedMeshData {
    ElementType element_type = VolumetricMesh::ElementType::Invalid;
    int num_element_vertices = 0;
    std::vector<Vec3d> vertices;
    std::vector<int> elements;
    std::vector<std::unique_ptr<VolumetricMesh::Material>> materials;
    std::vector<VolumetricMesh::Set> sets;
    std::vector<VolumetricMesh::Region> regions;
};

constexpr ElementType INVALID = ElementType::Invalid;
constexpr ElementType TET = ElementType::Tet;
constexpr ElementType CUBIC = ElementType::Cubic;
constexpr FileFormatType ASCII = FileFormatType::Ascii;
constexpr FileFormatType BINARY = FileFormatType::Binary;
constexpr FileFormatType BY_EXT = FileFormatType::ByExtension;
constexpr FileFormatType UNKNOWN = FileFormatType::Unknown;

template <typename T>
void write_exact(std::ostream& stream, const T* data, size_t count) {
    stream.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(sizeof(T) * count));
    if (!stream)
        throw std::runtime_error("Failed to write binary mesh data.");
}

template <typename T>
void read_exact(std::istream& stream, T* data, size_t count) {
    stream.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(sizeof(T) * count));
    if (!stream)
        throw std::runtime_error("Failed to read binary mesh data.");
}

ElementType element_type_from_int(int eleType) {
    switch (eleType) {
        case static_cast<int>(TET):
            return TET;
        case static_cast<int>(CUBIC):
            return CUBIC;
        default:
            return INVALID;
    }
}

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

LoadedMeshData make_loaded_mesh_data(RawLoadedMeshData raw, int verbose) {
    internal::VolumetricMeshData geometry(raw.num_element_vertices, std::move(raw.vertices), std::move(raw.elements));
    internal::MaterialCatalog material_catalog(std::move(raw.materials), std::move(raw.sets), std::move(raw.regions),
                                               geometry.num_elements(), verbose);
    return LoadedMeshData{raw.element_type, std::move(geometry), std::move(material_catalog)};
}

LoadedMeshData load_binary_data(std::istream& binaryInputStream) {
    RawLoadedMeshData data;

    double version = 0.0;
    read_exact(binaryInputStream, &version, 1);

    int eleType = 0;
    read_exact(binaryInputStream, &eleType, 1);
    data.element_type = element_type_from_int(eleType);
    if (data.element_type == INVALID) {
        printf("Error in io::load_binary_data: unknown mesh type %d in file stream\n", eleType);
        throw 2;
    }

    int numVertices = 0;
    read_exact(binaryInputStream, &numVertices, 1);
    if (numVertices < 0) {
        printf("Error in io::load_binary_data: incorrect number of vertices.\n");
        throw 3;
    }

    data.vertices.resize(static_cast<size_t>(numVertices));
    read_exact(binaryInputStream, reinterpret_cast<double*>(data.vertices.data()), static_cast<size_t>(3 * numVertices));

    int numElements = 0;
    read_exact(binaryInputStream, &numElements, 1);
    if (numElements < 0) {
        printf("Error in io::load_binary_data: incorrect number of elements.\n");
        throw 4;
    }

    read_exact(binaryInputStream, &data.num_element_vertices, 1);
    if (data.num_element_vertices <= 0) {
        printf("Error in io::load_binary_data: incorrect number of vertices per element.\n");
        throw 5;
    }

    data.elements.resize(static_cast<size_t>(numElements * data.num_element_vertices));
    read_exact(binaryInputStream, data.elements.data(), data.elements.size());

    int numMaterials = 0;
    read_exact(binaryInputStream, &numMaterials, 1);
    if (numMaterials < 0) {
        printf("Error in io::load_binary_data: incorrect number of materials.\n");
        throw 6;
    }

    data.materials.resize(static_cast<size_t>(numMaterials));
    for (int materialIndex = 0; materialIndex < numMaterials; materialIndex++) {
        data.materials[static_cast<size_t>(materialIndex)] = detail::read_binary_material(binaryInputStream);
    }

    int numSets = 0;
    read_exact(binaryInputStream, &numSets, 1);
    if (numSets <= 0) {
        printf("Error in io::load_binary_data: incorrect number of sets.\n");
        throw 10;
    }

    data.sets.resize(static_cast<size_t>(numSets));
    data.sets[0] = VolumetricMesh::generateAllElementsSet(numElements);

    std::vector<int> intTempVec;
    for (int setIndex = 1; setIndex < numSets; setIndex++) {
        char setName[4096];
        int length = 0;
        read_exact(binaryInputStream, &length, 1);
        read_exact(binaryInputStream, setName, static_cast<size_t>(length));
        setName[length] = '\0';
        data.sets[static_cast<size_t>(setIndex)] = VolumetricMesh::Set(setName);

        int cardinality = 0;
        read_exact(binaryInputStream, &cardinality, 1);
        intTempVec.resize(static_cast<size_t>(cardinality));
        read_exact(binaryInputStream, intTempVec.data(), intTempVec.size());
        for (int setElementIndex = 0; setElementIndex < cardinality; setElementIndex++)
            data.sets[static_cast<size_t>(setIndex)].insert(intTempVec[static_cast<size_t>(setElementIndex)]);
    }

    int numRegions = 0;
    read_exact(binaryInputStream, &numRegions, 1);
    if (numRegions < 0) {
        printf("Error in io::load_binary_data: incorrect number of regions.\n");
        throw 11;
    }

    data.regions.resize(static_cast<size_t>(numRegions));
    for (int regionIndex = 0; regionIndex < numRegions; regionIndex++) {
        int materialIndex = 0;
        int setIndex = 0;
        read_exact(binaryInputStream, &materialIndex, 1);
        read_exact(binaryInputStream, &setIndex, 1);
        data.regions[static_cast<size_t>(regionIndex)] = VolumetricMesh::Region(materialIndex, setIndex);
    }

    return make_loaded_mesh_data(std::move(data), 0);
}

LoadedMeshData load_binary_data(std::span<const std::byte> binaryInputStream) {
    if (binaryInputStream.empty())
        throw std::runtime_error("binary data buffer is empty");
    std::string buffer(reinterpret_cast<const char*>(binaryInputStream.data()), binaryInputStream.size());
    std::istringstream stream(buffer, std::ios::binary);
    return load_binary_data(stream);
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
        case BINARY: {
            std::ifstream stream(filename, std::ios::binary);
            if (!stream) {
                printf("Error in io::load_data: could not open file %s.\n", filename.string().c_str());
                throw 1;
            }
            return load_binary_data(stream);
        }
        default:
            printf("Error in io::load_data: file format is unknown.\n");
            throw 1;
    }
}

void save_binary_impl(const VolumetricMesh& mesh, std::ostream& out, unsigned int* bytesWritten) {
    unsigned int totalBytesWritten = 0;

    auto count_write = [&](const auto* data, size_t count) {
        using T = std::remove_pointer_t<decltype(data)>;
        write_exact(out, data, count);
        totalBytesWritten += static_cast<unsigned int>(sizeof(T) * count);
    };

    const double version = 1.0;
    count_write(&version, 1);

    const int eleType = static_cast<int>(mesh.getElementType());
    count_write(&eleType, 1);
    const int numVertices = mesh.getNumVertices();
    count_write(&numVertices, 1);

    for (int vertexIndex = 0; vertexIndex < mesh.getNumVertices(); vertexIndex++) {
        const Vec3d& v = mesh.getVertex(vertexIndex);
        count_write(v.data(), 3);
    }

    const int numElements = mesh.getNumElements();
    const int numElementVertices = mesh.getNumElementVertices();
    count_write(&numElements, 1);
    count_write(&numElementVertices, 1);

    for (int elementIndex = 0; elementIndex < mesh.getNumElements(); elementIndex++) {
        const auto indices = mesh.getVertexIndices(elementIndex);
        count_write(indices.data(), static_cast<size_t>(mesh.getNumElementVertices()));
    }

    const int numMaterials = mesh.getNumMaterials();
    count_write(&numMaterials, 1);
    for (int materialIndex = 0; materialIndex < numMaterials; materialIndex++) {
        totalBytesWritten += detail::write_binary_material(out, *mesh.getMaterial(materialIndex));
    }

    const int numSets = mesh.getNumSets();
    count_write(&numSets, 1);
    for (int setIndex = 1; setIndex < numSets; setIndex++) {
        const auto& set = mesh.getSet(setIndex);
        const unsigned int length = static_cast<unsigned int>(set.getName().size());
        count_write(&length, 1);
        out.write(set.getName().c_str(), static_cast<std::streamsize>(length));
        if (!out)
            throw std::runtime_error("Failed to write set name.");
        totalBytesWritten += length;

        const unsigned int cardinality = static_cast<unsigned int>(set.getNumElements());
        count_write(&cardinality, 1);
        std::vector<int> elementGroup(set.getElements().begin(), set.getElements().end());
        count_write(elementGroup.data(), elementGroup.size());
    }

    const int numRegions = mesh.getNumRegions();
    count_write(&numRegions, 1);
    for (int regionIndex = 0; regionIndex < mesh.getNumRegions(); regionIndex++) {
        const auto& region = mesh.getRegion(regionIndex);
        const int materialIndex = region.getMaterialIndex();
        const int setIndex = region.getSetIndex();
        count_write(&materialIndex, 1);
        count_write(&setIndex, 1);
    }

    if (bytesWritten != nullptr)
        *bytesWritten = totalBytesWritten;
}

}  // namespace

namespace detail {

LoadedMeshData load_tet_data(const std::filesystem::path& filename, VolumetricMesh::FileFormatType fileFormat,
                             int verbose) {
    LoadedMeshData data = load_data(filename, fileFormat, verbose);
    validate_loaded_data(data, TET);
    return data;
}

LoadedMeshData load_tet_data(std::span<const std::byte> binaryData) {
    LoadedMeshData data = load_binary_data(binaryData);
    validate_loaded_data(data, TET);
    return data;
}

LoadedMeshData load_cubic_data(const std::filesystem::path& filename, VolumetricMesh::FileFormatType fileFormat,
                               int verbose) {
    LoadedMeshData data = load_data(filename, fileFormat, verbose);
    validate_loaded_data(data, CUBIC);
    return data;
}

LoadedMeshData load_cubic_data(std::span<const std::byte> binaryData) {
    LoadedMeshData data = load_binary_data(binaryData);
    validate_loaded_data(data, CUBIC);
    return data;
}

}  // namespace detail

int save(const VolumetricMesh& mesh, const std::filesystem::path& filename) {
    const FileFormatType fileType = detect_file_format(filename);
    if (fileType == FileFormatType::Binary)
        return save_to_binary(mesh, filename);
    return save_to_ascii(mesh, filename);
}

int save_to_ascii(const VolumetricMesh& mesh, const std::filesystem::path& filename) {
    try {
        detail::write_ascii_mesh(mesh, filename);
        return 0;
    } catch (...) {
        return 1;
    }
}

int save_to_binary(const VolumetricMesh& mesh, const std::filesystem::path& filename, unsigned int* bytesWritten) {
    try {
        std::ofstream out(filename, std::ios::binary);
        if (!out) {
            printf("Error: could not write to %s.\n", filename.string().c_str());
            return 1;
        }
        save_binary_impl(mesh, out, bytesWritten);
        return 0;
    } catch (...) {
        return 1;
    }
}

template <typename Mesh>
std::unique_ptr<Mesh> load_impl(const std::filesystem::path& filename, VolumetricMesh::FileFormatType fileFormat,
                                int verbose) {
    return std::make_unique<Mesh>(filename, fileFormat, verbose);
}

std::unique_ptr<TetMesh> load_tet(const std::filesystem::path& filename, VolumetricMesh::FileFormatType fileFormat,
                                  int verbose) {
    return load_impl<TetMesh>(filename, fileFormat, verbose);
}

std::unique_ptr<CubicMesh> load_cubic(const std::filesystem::path& filename,
                                      VolumetricMesh::FileFormatType fileFormat, int verbose) {
    return load_impl<CubicMesh>(filename, fileFormat, verbose);
}

}  // namespace io
}  // namespace pgo::VolumetricMeshes
