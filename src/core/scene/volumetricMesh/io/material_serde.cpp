#include "io/material_serde.h"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace pgo::VolumetricMeshes::io::detail {
namespace {

constexpr int ENU_TAG = 1;
constexpr int ORTHOTROPIC_TAG = 2;
constexpr int MOONEYRIVLIN_TAG = 3;

constexpr int ENU_NUM_PROPERTIES = 3;
constexpr int ORTHOTROPIC_NUM_PROPERTIES = 10;
constexpr int MOONEYRIVLIN_NUM_PROPERTIES = 4;

template <typename T>
void read_exact(std::istream& input, T* data, size_t count) {
    input.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(sizeof(T) * count));
    if (!input) {
        throw std::runtime_error("Failed to read binary mesh data.");
    }
}

template <typename T>
void write_exact(std::ostream& output, const T* data, size_t count) {
    output.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(sizeof(T) * count));
    if (!output) {
        throw std::runtime_error("Failed to write binary mesh data.");
    }
}

MaterialRecord parse_orthotropic_material(const std::string& material_name, const char* material_type, char* parameters,
                                          const std::filesystem::path& filename, const std::string& offending_line) {
    OrthotropicMaterialData data;
    double nu = 0.0;
    double G = 1.0;
    bool use_nu_and_g = false;

    char* sub_type = const_cast<char*>(material_type) + 11;
    bool enough_parameters = false;

    if ((sscanf(parameters, "%lf,%lf,%lf,%lf", &data.density, &data.E1, &data.E2, &data.E3) == 4) && (data.E1 > 0) &&
        (data.E2 > 0) && (data.E3 > 0) && (data.density > 0)) {
        for (int i = 0; i < 4; ++i) {
            while ((*parameters != ',') && (*parameters != 0)) {
                parameters++;
            }
            if (*parameters == 0) {
                break;
            }
            parameters++;
        }

        if ((*sub_type == 0) || (strcmp(sub_type, "_N3G3R9") == 0)) {
            if (sscanf(parameters,
                       "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                       &data.nu12, &data.nu23, &data.nu31, &data.G12, &data.G23, &data.G31, &data.rotation[0],
                       &data.rotation[1], &data.rotation[2], &data.rotation[3], &data.rotation[4], &data.rotation[5],
                       &data.rotation[6], &data.rotation[7], &data.rotation[8]) == 15) {
                enough_parameters = true;
            }
        } else if (strcmp(sub_type, "_N3G3") == 0) {
            if (sscanf(parameters, "%lf,%lf,%lf,%lf,%lf,%lf", &data.nu12, &data.nu23, &data.nu31, &data.G12,
                       &data.G23, &data.G31) == 6) {
                enough_parameters = true;
            }
        } else if (strcmp(sub_type, "_N1G1R9") == 0) {
            if (sscanf(parameters, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &nu, &G, &data.rotation[0],
                       &data.rotation[1], &data.rotation[2], &data.rotation[3], &data.rotation[4], &data.rotation[5],
                       &data.rotation[6], &data.rotation[7], &data.rotation[8]) == 11) {
                use_nu_and_g = true;
                enough_parameters = true;
            }
        } else if (strcmp(sub_type, "_N1G1") == 0) {
            if (sscanf(parameters, "%lf,%lf", &nu, &G) == 2) {
                use_nu_and_g = true;
                enough_parameters = true;
            }
        } else if (strcmp(sub_type, "_N1R9") == 0) {
            if (sscanf(parameters, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &nu, &data.rotation[0],
                       &data.rotation[1], &data.rotation[2], &data.rotation[3], &data.rotation[4], &data.rotation[5],
                       &data.rotation[6], &data.rotation[7], &data.rotation[8]) == 10) {
                use_nu_and_g = true;
                enough_parameters = true;
            }
        } else if (strcmp(sub_type, "_N1") == 0) {
            if (sscanf(parameters, "%lf", &nu) == 1) {
                use_nu_and_g = true;
                enough_parameters = true;
            }
        } else {
            printf("Error: incorrect orthortropic material type \"%s\" in file %s. Offending line: %s\n", sub_type,
                   filename.string().c_str(), offending_line.c_str());
            throw 14;
        }

        if (use_nu_and_g && (nu > -1.0) && (nu < 0.5)) {
            data.nu12 = nu * sqrt(data.E1 / data.E2);
            data.nu23 = nu * sqrt(data.E2 / data.E3);
            data.nu31 = nu * sqrt(data.E3 / data.E1);
            data.G12 = G * sqrt(data.E1 * data.E2) / (2.0 * (1.0 + nu));
            data.G23 = G * sqrt(data.E2 * data.E3) / (2.0 * (1.0 + nu));
            data.G31 = G * sqrt(data.E3 * data.E1) / (2.0 * (1.0 + nu));
        }
    }

    if (enough_parameters && (data.G12 > 0) && (data.G23 > 0) && (data.G31 > 0)) {
        return MaterialRecord{material_name, data};
    }

    printf("Error: incorrect material specification in file %s. Offending line: %s\n", filename.string().c_str(),
           offending_line.c_str());
    throw 14;
}

}  // namespace

MaterialRecord parse_ascii_material(const std::string& material_name, const std::string& material_specification,
                                    const std::filesystem::path& filename, const std::string& offending_line) {
    char material_specification_buffer[4096];
    strncpy(material_specification_buffer, material_specification.c_str(), sizeof(material_specification_buffer) - 1);
    material_specification_buffer[sizeof(material_specification_buffer) - 1] = '\0';

    char* parameters = material_specification_buffer;
    while ((*parameters != ',') && (*parameters != 0)) {
        parameters++;
    }

    if (*parameters == 0) {
        printf("Error parsing file %s. Offending line: %s.\n", filename.string().c_str(), offending_line.c_str());
        throw 12;
    }

    char material_type[4096];
    const unsigned int material_type_length = static_cast<unsigned int>(parameters - material_specification_buffer);
    memcpy(material_type, material_specification_buffer, sizeof(unsigned char) * material_type_length);
    material_type[material_type_length] = 0;

    parameters++;

    if (strcmp(material_type, "ENU") == 0) {
        EnuMaterialData data;
        sscanf(parameters, "%lf,%lf,%lf", &data.density, &data.E, &data.nu);

        if ((data.E > 0) && (data.nu > -1.0) && (data.nu < 0.5) && (data.density > 0)) {
            return MaterialRecord{material_name, data};
        }

        printf("Error: incorrect material specification in file %s. Offending line: %s\n",
               filename.string().c_str(), offending_line.c_str());
        throw 13;
    }

    if (strncmp(material_type, "ORTHOTROPIC", 11) == 0) {
        return parse_orthotropic_material(material_name, material_type, parameters, filename, offending_line);
    }

    if (strncmp(material_type, "MOONEYRIVLIN", 12) == 0) {
        MooneyRivlinMaterialData data;
        sscanf(parameters, "%lf,%lf,%lf,%lf", &data.density, &data.mu01, &data.mu10, &data.v1);

        if (data.density > 0) {
            return MaterialRecord{material_name, data};
        }

        printf("Error: incorrect material specification in file %s. Offending line:\n%s\n",
               filename.string().c_str(), offending_line.c_str());
        throw 15;
    }

    printf("Error: incorrect material specification in file %s. Offending line:\n%s\n", filename.string().c_str(),
           offending_line.c_str());
    throw 16;
}

MaterialRecord read_binary_material(std::istream& input) {
    char material_name[4096];
    int length = 0;
    read_exact(input, &length, 1);
    read_exact(input, material_name, static_cast<size_t>(length));
    material_name[length] = '\0';

    int material_type = 0;
    read_exact(input, &material_type, 1);

    switch (material_type) {
        case ENU_TAG: {
            double properties[ENU_NUM_PROPERTIES];
            read_exact(input, properties, ENU_NUM_PROPERTIES);
            if ((properties[1] > 0) && (properties[2] > -1.0) && (properties[2] < 0.5) && (properties[0] > 0)) {
                return MaterialRecord{material_name, EnuMaterialData{properties[0], properties[1], properties[2]}};
            }
            printf("Error in io::load_binary_data: incorrect material specification in file stream.\n");
            throw 7;
        }
        case ORTHOTROPIC_TAG: {
            double properties[ORTHOTROPIC_NUM_PROPERTIES];
            std::array<double, 9> rotation;
            read_exact(input, properties, ORTHOTROPIC_NUM_PROPERTIES);
            read_exact(input, rotation.data(), rotation.size());
            if ((properties[1] > 0) && (properties[2] > 0) && (properties[3] > 0) && (properties[7] > 0) &&
                (properties[8] > 0) && (properties[9] > 0) && (properties[0] > 0)) {
                return MaterialRecord{
                    material_name,
                    OrthotropicMaterialData{properties[0], properties[1], properties[2], properties[3], properties[4],
                                            properties[5], properties[6], properties[7], properties[8], properties[9],
                                            rotation}};
            }
            printf("Error in io::load_binary_data: incorrect orthotropic material specification.\n");
            throw 14;
        }
        case MOONEYRIVLIN_TAG: {
            double properties[MOONEYRIVLIN_NUM_PROPERTIES];
            read_exact(input, properties, MOONEYRIVLIN_NUM_PROPERTIES);
            if (properties[0] > 0) {
                return MaterialRecord{material_name, MooneyRivlinMaterialData{properties[0], properties[1], properties[2], properties[3]}};
            }
            printf("Error in io::load_binary_data: incorrect Mooney-Rivlin material specification.\n");
            throw 8;
        }
        default:
            printf("Error in io::load_binary_data: material type %d is unknown.\n", material_type);
            throw 9;
    }
}

void write_ascii_material(FILE* output, const MaterialRecord& material) {
    fprintf(output, "*MATERIAL %s\n", material.name.c_str());
    visit_material(material, [&](const auto& value) {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, EnuMaterialData>) {
            fprintf(output, "ENU, %.15G, %.15G, %.15G\n", value.density, value.E, value.nu);
        } else if constexpr (std::is_same_v<T, OrthotropicMaterialData>) {
            fprintf(output,
                    "ORTHOTROPIC, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, "
                    "%.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G\n",
                    value.density, value.E1, value.E2, value.E3, value.nu12, value.nu23, value.nu31, value.G12,
                    value.G23, value.G31, value.rotation[0], value.rotation[1], value.rotation[2], value.rotation[3],
                    value.rotation[4], value.rotation[5], value.rotation[6], value.rotation[7], value.rotation[8]);
        } else {
            fprintf(output, "MOONEYRIVLIN, %.15G, %.15G, %.20G, %.15G\n", value.density, value.mu01, value.mu10,
                    value.v1);
        }
    });
    fprintf(output, "\n");
}

unsigned int write_binary_material(std::ostream& output, const MaterialRecord& material) {
    unsigned int bytes_written = 0;
    auto count_write = [&](const auto* data, size_t count) {
        using T = std::remove_pointer_t<decltype(data)>;
        write_exact(output, data, count);
        bytes_written += static_cast<unsigned int>(sizeof(T) * count);
    };

    const unsigned int length = static_cast<unsigned int>(material.name.size());
    count_write(&length, 1);
    output.write(material.name.c_str(), static_cast<std::streamsize>(length));
    if (!output) {
        throw std::runtime_error("Failed to write material name.");
    }
    bytes_written += length;

    visit_material(material, [&](const auto& value) {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, EnuMaterialData>) {
            const int tag = ENU_TAG;
            const double properties[] = {value.density, value.E, value.nu};
            count_write(&tag, 1);
            count_write(properties, 3);
        } else if constexpr (std::is_same_v<T, OrthotropicMaterialData>) {
            const int tag = ORTHOTROPIC_TAG;
            const double properties[] = {value.density, value.E1, value.E2, value.E3, value.nu12,
                                         value.nu23,   value.nu31, value.G12, value.G23, value.G31};
            count_write(&tag, 1);
            count_write(properties, 10);
            count_write(value.rotation.data(), value.rotation.size());
        } else {
            const int tag = MOONEYRIVLIN_TAG;
            const double properties[] = {value.density, value.mu01, value.mu10, value.v1};
            count_write(&tag, 1);
            count_write(properties, 4);
        }
    });

    return bytes_written;
}

}  // namespace pgo::VolumetricMeshes::io::detail
