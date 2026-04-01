#include "io/mesh_ascii_reader.h"

#include "io/detail/veg_parser.h"
#include "io/material_serde.h"
#include "stringHelper.h"

#include <cstdlib>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <map>
#include <utility>

namespace pgo::VolumetricMeshes::io::detail {
namespace {

struct RawLoadedMeshData {
    ElementType element_type = ElementType::Invalid;
    int num_element_vertices = 0;
    std::vector<Vec3d> vertices;
    std::vector<int> elements;
    std::vector<MaterialRecord> materials;
    std::vector<ElementSet> sets;
    std::vector<MaterialRegion> regions;
};

constexpr ElementType kTet = ElementType::Tet;
constexpr ElementType kCubic = ElementType::Cubic;

int vertices_per_element(ElementType element_type) {
    switch (element_type) {
        case kTet:
            return 4;
        case kCubic:
            return 8;
        default:
            return 0;
    }
}

LoadedMeshData make_loaded_mesh_data(RawLoadedMeshData raw, int verbose) {
    internal::VolumetricMeshData geometry(raw.num_element_vertices, std::move(raw.vertices), std::move(raw.elements));
    internal::MaterialCatalog material_catalog(std::move(raw.materials), std::move(raw.sets), std::move(raw.regions),
                                               geometry.num_elements(), verbose);
    return LoadedMeshData{raw.element_type, std::move(geometry), std::move(material_catalog)};
}

}  // namespace

LoadedMeshData read_ascii_mesh(const std::filesystem::path& filename, int verbose) {
    const std::string filename_string = filename.string();
    const char* filename_c_str = filename_string.c_str();

    VolumetricMeshParser parser;
    if (parser.open(filename_c_str) != 0) {
        printf("Error: could not open file %s.\n", filename_c_str);
        throw 1;
    }

    RawLoadedMeshData data;

    int count_num_vertices = 0;
    int count_num_elements = 0;
    int num_vertices = -1;
    int num_elements = -1;
    int num_materials = 0;
    int num_sets = 1;
    int num_regions = 0;
    int parse_state = 0;
    int one_indexed_vertices = 1;
    int one_indexed_elements = 1;
    char line_buffer[1024];

    while (parser.getNextLine(line_buffer, 0, 0) != nullptr) {
        if ((parse_state == 0) && (strncmp(line_buffer, "*VERTICES", 9) == 0)) {
            parse_state = 1;

            if (parser.getNextLine(line_buffer, 0, 0) != nullptr) {
                sscanf(line_buffer, "%d", &num_vertices);
                data.vertices.resize(static_cast<size_t>(num_vertices));
            } else {
                printf("Error: file %s is not in the .veg format. Offending line:\n%s\n", filename_c_str, line_buffer);
                throw 2;
            }
            continue;
        }

        if ((parse_state == 1) && (strncmp(line_buffer, "*ELEMENTS", 9) == 0)) {
            parse_state = 2;

            if (parser.getNextLine(line_buffer) != nullptr) {
                parser.removeWhitespace(line_buffer);

                if (strncmp(line_buffer, "TET", 3) == 0) {
                    data.element_type = kTet;
                } else if (strncmp(line_buffer, "CUBIC", 5) == 0) {
                    data.element_type = kCubic;
                } else {
                    printf("Error: unknown mesh type %s in file %s\n", line_buffer, filename_c_str);
                    throw 3;
                }

                data.num_element_vertices = vertices_per_element(data.element_type);
            } else {
                printf("Error: file %s is not in the .veg format. Offending line:\n%s\n", filename_c_str, line_buffer);
                throw 4;
            }

            if (parser.getNextLine(line_buffer, 0, 0) != nullptr) {
                sscanf(line_buffer, "%d", &num_elements);
                data.elements.resize(static_cast<size_t>(num_elements * data.num_element_vertices));
            } else {
                printf("Error: file %s is not in the .veg format. Offending line:\n%s\n", filename_c_str, line_buffer);
                throw 5;
            }
            continue;
        }

        if ((parse_state == 2) && (line_buffer[0] == '*')) {
            parse_state = 3;
        }

        if (parse_state == 1) {
            if (count_num_vertices >= num_vertices) {
                printf("Error: mismatch in the number of vertices in %s.\n", filename_c_str);
                throw 6;
            }

            char* ch = line_buffer;
            while ((*ch == ' ') || (*ch == ',') || (*ch == '\t')) {
                ch++;
            }

            int index = -1;
            sscanf(ch, "%d", &index);
            while ((*ch != ' ') && (*ch != ',') && (*ch != '\t') && (*ch != 0)) {
                ch++;
            }

            if (index == 0) {
                one_indexed_vertices = 0;
            }

            double pos[3];
            for (int i = 0; i < 3; i++) {
                while ((*ch == ' ') || (*ch == ',') || (*ch == '\t')) {
                    ch++;
                }

                if (*ch == 0) {
                    printf("Error parsing line %s in file %s.\n", line_buffer, filename_c_str);
                    throw 7;
                }

                sscanf(ch, "%lf", &pos[i]);
                while ((*ch != ' ') && (*ch != ',') && (*ch != '\t') && (*ch != 0)) {
                    ch++;
                }
            }

            data.vertices[static_cast<size_t>(count_num_vertices)] = Vec3d(pos);
            count_num_vertices++;
        }

        if (parse_state == 2) {
            if (count_num_elements >= num_elements) {
                printf("Error: mismatch in the number of elements in %s.\n", filename_c_str);
                throw 8;
            }

            char* ch = line_buffer;
            while ((*ch == ' ') || (*ch == ',') || (*ch == '\t')) {
                ch++;
            }

            int index = -1;
            sscanf(ch, "%d", &index);
            if (index == 0) {
                one_indexed_elements = 0;
            }

            while ((*ch != ' ') && (*ch != ',') && (*ch != '\t') && (*ch != 0)) {
                ch++;
            }

            for (int i = 0; i < data.num_element_vertices; i++) {
                while ((*ch == ' ') || (*ch == ',') || (*ch == '\t')) {
                    ch++;
                }

                if (*ch == 0) {
                    printf("Error parsing line %s in file %s.\n", line_buffer, filename_c_str);
                    throw 9;
                }

                int vertex_index = -1;
                sscanf(ch, "%d", &vertex_index);
                data.elements[static_cast<size_t>(count_num_elements * data.num_element_vertices + i)] =
                    vertex_index - one_indexed_vertices;

                while ((*ch != ' ') && (*ch != ',') && (*ch != '\t') && (*ch != 0)) {
                    ch++;
                }
            }

            count_num_elements++;
        }

        if (strncmp(line_buffer, "*MATERIAL", 9) == 0) {
            num_materials++;
        }
        if (strncmp(line_buffer, "*SET", 4) == 0) {
            num_sets++;
        }
        if (strncmp(line_buffer, "*REGION", 7) == 0) {
            num_regions++;
        }
    }

    if (num_elements < 0) {
        printf("Error: incorrect number of elements. File %s may not be in the .veg format.\n", filename_c_str);
        throw 10;
    }

    parser.rewindToStart();

    if (verbose) {
        if (num_materials == 0) {
            printf("Warning: no materials encountered in %s.\n", filename_c_str);
        }
        if (num_regions == 0) {
            printf("Warning: no regions encountered in %s.\n", filename_c_str);
        }
    }

    data.materials.resize(static_cast<size_t>(num_materials));
    data.sets.resize(static_cast<size_t>(num_sets));
    data.regions.resize(static_cast<size_t>(num_regions));
    data.sets[0] = VolumetricMesh::generateAllElementsSet(num_elements);

    int count_num_materials = 0;
    int count_num_sets = 1;
    int count_num_regions = 0;

    std::map<std::string, int> material_map;
    std::map<std::string, int> set_map;
    set_map.insert(std::pair<std::string, int>(data.sets[0].getName(), 0));

    parse_state = 0;

    while (parser.getNextLine(line_buffer, 0, 0) != nullptr) {
        if ((parse_state == 11) && (line_buffer[0] == '*')) {
            parse_state = 0;
        }

        if ((parse_state == 0) && (strncmp(line_buffer, "*MATERIAL", 9) == 0)) {
            parser.removeWhitespace(line_buffer);

            char material_name_c[4096];
            strcpy(material_name_c, &line_buffer[9]);

            char material_specification[4096];
            if (parser.getNextLine(line_buffer) != nullptr) {
                parser.removeWhitespace(line_buffer);
                sscanf(line_buffer, "%s", material_specification);
            } else {
                printf("Error: incorrect material in file %s. Offending line:\n%s\n", filename_c_str, line_buffer);
                throw 11;
            }

            char* ch = material_specification;
            while ((*ch != ',') && (*ch != 0)) {
                ch++;
            }

            if (*ch == 0) {
                printf("Error parsing file %s. Offending line: %s.\n", filename_c_str, line_buffer);
                throw 12;
            }

            std::string name(material_name_c);
            data.materials[static_cast<size_t>(count_num_materials)] =
                parse_ascii_material(name, material_specification, filename, line_buffer);
            material_map.insert(std::pair<std::string, int>(name, count_num_materials));

            count_num_materials++;
        }

        if ((parse_state == 0) && (strncmp(line_buffer, "*REGION", 7) == 0)) {
            parser.removeWhitespace(line_buffer);

            char set_name_c[4096];
            char material_name_c[4096];

            if (parser.getNextLine(line_buffer) != nullptr) {
                parser.removeWhitespace(line_buffer);

                char* ch = line_buffer;
                while ((*ch != ',') && (*ch != 0)) {
                    ch++;
                }

                if (*ch == 0) {
                    printf("Error parsing file %s. Offending line: %s.\n", filename_c_str, line_buffer);
                    throw 17;
                }

                *ch = 0;
                strcpy(set_name_c, line_buffer);
                *ch = ',';
                ch++;
                strcpy(material_name_c, ch);
            } else {
                printf("Error: file %s is not in the .veg format. Offending line:\n%s\n", filename_c_str, line_buffer);
                throw 18;
            }

            int set_num = -1;
            auto it = set_map.find(std::string(set_name_c));
            if (it != set_map.end()) {
                set_num = it->second;
            } else {
                printf("Error: set \"%s\" not found among the sets.\n", set_name_c);
                printf("All existing sets:\n");
                for (const auto& p : set_map) {
                    printf("%i: \"%s\"\n", p.second, p.first.c_str());
                }
                printf("\n");
                throw 19;
            }

            int material_num = -1;
            it = material_map.find(std::string(material_name_c));
            if (it != material_map.end()) {
                material_num = it->second;
            } else {
                printf("Error: material %s not found among the materials.\n", material_name_c);
                throw 20;
            }

            data.regions[static_cast<size_t>(count_num_regions)] = MaterialRegion(material_num, set_num);
            count_num_regions++;
        }

        if (parse_state == 11) {
            parser.removeWhitespace(line_buffer);

            char* pch = strtok(line_buffer, ",");
            while ((pch != nullptr) && (isdigit(*pch))) {
                int new_element = atoi(pch);
                int index = new_element - one_indexed_elements;
                if (index >= num_elements || index < 0) {
                    printf("Error: set element index: %d out of bounds.\n", new_element);
                    throw 21;
                }
                data.sets[static_cast<size_t>(count_num_sets - 1)].insert(index);
                pch = strtok(nullptr, ",");
            }
        }

        if ((parse_state == 0) && (strncmp(line_buffer, "*SET", 4) == 0)) {
            parser.removeWhitespace(line_buffer);

            std::string name(BasicAlgorithms::stripLight(&line_buffer[4]));
            data.sets[static_cast<size_t>(count_num_sets)] = ElementSet(name);
            set_map.insert(std::pair<std::string, int>(name, count_num_sets));
            count_num_sets++;
            parse_state = 11;
        }
    }

    parser.close();
    return make_loaded_mesh_data(std::move(data), verbose);
}

}  // namespace pgo::VolumetricMeshes::io::detail
