#include "io/mesh_format_detector.h"

#include "volumetricMeshParser.h"

#include "stringHelper.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace pgo::VolumetricMeshes::io {
namespace {

using ElementType = VolumetricMesh::ElementType;
using FileFormatType = VolumetricMesh::FileFormatType;

constexpr ElementType INVALID = ElementType::Invalid;
constexpr ElementType TET = ElementType::Tet;
constexpr ElementType CUBIC = ElementType::Cubic;
constexpr FileFormatType ASCII = FileFormatType::Ascii;
constexpr FileFormatType BINARY = FileFormatType::Binary;
constexpr FileFormatType BY_EXT = FileFormatType::ByExtension;
constexpr FileFormatType UNKNOWN = FileFormatType::Unknown;

template <typename T>
void read_exact(std::istream& stream, T* data, size_t count) {
    stream.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(sizeof(T) * count));
    if (!stream) {
        throw std::runtime_error("Failed to read binary mesh data.");
    }
}

ElementType element_type_from_int(int ele_type) {
    switch (ele_type) {
        case static_cast<int>(TET):
            return TET;
        case static_cast<int>(CUBIC):
            return CUBIC;
        default:
            return INVALID;
    }
}

}  // namespace

namespace detail {

ElementType detect_ascii_element_type(const std::filesystem::path& filename) {
    VolumetricMeshParser parser;
    if (parser.open(filename.string().c_str()) != 0) {
        printf("Error: could not open file %s.\n", filename.string().c_str());
        return INVALID;
    }

    char line_buffer[1024];
    ElementType result = INVALID;
    while (parser.getNextLine(line_buffer, 0, 0) != nullptr) {
        if (strncmp(line_buffer, "*ELEMENTS", 9) == 0) {
            if (parser.getNextLine(line_buffer) != nullptr) {
                parser.removeWhitespace(line_buffer);
                if (strncmp(line_buffer, "TET", 3) == 0) {
                    result = TET;
                } else if (strncmp(line_buffer, "CUBIC", 5) == 0) {
                    result = CUBIC;
                }
            }
            break;
        }
    }
    parser.close();
    return result;
}

ElementType detect_binary_element_type(std::istream& input) {
    double version = 0.0;
    int ele_type = 0;
    read_exact(input, &version, 1);
    read_exact(input, &ele_type, 1);
    return element_type_from_int(ele_type);
}

FileFormatType detect_file_format_by_ext(const std::filesystem::path& filename) {
    const std::string filename_string = filename.string();
    if (BasicAlgorithms::iendWith(filename_string.c_str(), ".vegb")) {
        return FileFormatType::Binary;
    }
    if (BasicAlgorithms::iendWith(filename_string.c_str(), ".veg")) {
        return FileFormatType::Ascii;
    }
    return FileFormatType::Unknown;
}

}  // namespace detail

VolumetricMesh::ElementType detect_element_type(const std::filesystem::path& filename,
                                                VolumetricMesh::FileFormatType file_format) {
    if (file_format == BY_EXT) {
        file_format = detect_file_format(filename);
        if (file_format == UNKNOWN) {
            printf("Unknown file extension when loading %s, try ASCII format...\n", filename.string().c_str());
            file_format = ASCII;
        }
    }

    if (file_format == ASCII) {
        return detail::detect_ascii_element_type(filename);
    }

    if (file_format == BINARY) {
        std::ifstream input(filename, std::ios::binary);
        if (!input) {
            printf("Error in io::detect_element_type: could not open file %s.\n", filename.string().c_str());
            return INVALID;
        }
        return detail::detect_binary_element_type(input);
    }

    printf("Error: the file format %d is unknown.\n", static_cast<int>(file_format));
    return INVALID;
}

VolumetricMesh::ElementType detect_element_type(std::span<const std::byte> binary_data) {
    if (binary_data.empty()) {
        throw std::runtime_error("binary data buffer is empty");
    }
    std::string buffer(reinterpret_cast<const char*>(binary_data.data()), binary_data.size());
    std::istringstream input(buffer, std::ios::binary);
    return detail::detect_binary_element_type(input);
}

VolumetricMesh::FileFormatType detect_file_format(const std::filesystem::path& filename) {
    return detail::detect_file_format_by_ext(filename);
}

}  // namespace pgo::VolumetricMeshes::io
