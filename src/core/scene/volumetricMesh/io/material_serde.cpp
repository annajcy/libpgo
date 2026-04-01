#include "io/material_serde.h"

#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshMooneyRivlinMaterial.h"
#include "volumetricMeshOrthotropicMaterial.h"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace pgo::VolumetricMeshes::io::detail {
namespace {

using MaterialType = VolumetricMesh::Material::MaterialType;
using ENuMaterial = VolumetricMesh::ENuMaterial;
using OrthotropicMaterial = VolumetricMesh::OrthotropicMaterial;
using MooneyRivlinMaterial = VolumetricMesh::MooneyRivlinMaterial;

constexpr int ENU_DENSITY = static_cast<int>(VolumetricMesh::Material::ENuMaterialProperty::Density);
constexpr int ENU_E = static_cast<int>(VolumetricMesh::Material::ENuMaterialProperty::E);
constexpr int ENU_NU = static_cast<int>(VolumetricMesh::Material::ENuMaterialProperty::Nu);
constexpr int ENU_NUM_PROPERTIES = static_cast<int>(VolumetricMesh::Material::ENuMaterialProperty::Count);

constexpr int ORTHOTROPIC_DENSITY = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::Density);
constexpr int ORTHOTROPIC_E1 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::E1);
constexpr int ORTHOTROPIC_E2 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::E2);
constexpr int ORTHOTROPIC_E3 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::E3);
constexpr int ORTHOTROPIC_NU12 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::Nu12);
constexpr int ORTHOTROPIC_NU23 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::Nu23);
constexpr int ORTHOTROPIC_NU31 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::Nu31);
constexpr int ORTHOTROPIC_G12 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::G12);
constexpr int ORTHOTROPIC_G23 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::G23);
constexpr int ORTHOTROPIC_G31 = static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::G31);
constexpr int ORTHOTROPIC_NUM_PROPERTIES =
    static_cast<int>(VolumetricMesh::Material::OrthotropicMaterialProperty::Count);

constexpr int MOONEYRIVLIN_DENSITY = static_cast<int>(VolumetricMesh::Material::MooneyRivlinMaterialProperty::Density);
constexpr int MOONEYRIVLIN_MU01 = static_cast<int>(VolumetricMesh::Material::MooneyRivlinMaterialProperty::Mu01);
constexpr int MOONEYRIVLIN_MU10 = static_cast<int>(VolumetricMesh::Material::MooneyRivlinMaterialProperty::Mu10);
constexpr int MOONEYRIVLIN_V1 = static_cast<int>(VolumetricMesh::Material::MooneyRivlinMaterialProperty::V1);
constexpr int MOONEYRIVLIN_NUM_PROPERTIES =
    static_cast<int>(VolumetricMesh::Material::MooneyRivlinMaterialProperty::Count);

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

std::unique_ptr<VolumetricMesh::Material> parse_orthotropic_material(const std::string& material_name,
                                                                     const char* material_type,
                                                                     char* parameters,
                                                                     const std::filesystem::path& filename,
                                                                     const std::string& offending_line) {
    double density = 0.0;
    double E1 = 0.0;
    double E2 = 0.0;
    double E3 = 0.0;
    double nu12 = 0.0;
    double nu23 = 0.0;
    double nu31 = 0.0;
    double G12 = 0.0;
    double G23 = 0.0;
    double G31 = 0.0;
    double nu = 0.0;
    double G = 1.0;
    bool use_nu_and_g = false;
    double rotation[9];
    memset(rotation, 0, sizeof(rotation));
    rotation[0] = rotation[4] = rotation[8] = 1.0;

    char* sub_type = const_cast<char*>(material_type) + 11;
    bool enough_parameters = false;

    if ((sscanf(parameters, "%lf,%lf,%lf,%lf", &density, &E1, &E2, &E3) == 4) &&
        (E1 > 0) && (E2 > 0) && (E3 > 0) && (density > 0)) {
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
                       &nu12, &nu23, &nu31, &G12, &G23, &G31, &rotation[0], &rotation[1], &rotation[2], &rotation[3],
                       &rotation[4], &rotation[5], &rotation[6], &rotation[7], &rotation[8]) == 15) {
                enough_parameters = true;
            }
        } else if (strcmp(sub_type, "_N3G3") == 0) {
            if (sscanf(parameters, "%lf,%lf,%lf,%lf,%lf,%lf", &nu12, &nu23, &nu31, &G12, &G23, &G31) == 6) {
                enough_parameters = true;
            }
        } else if (strcmp(sub_type, "_N1G1R9") == 0) {
            if (sscanf(parameters, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &nu, &G, &rotation[0],
                       &rotation[1], &rotation[2], &rotation[3], &rotation[4], &rotation[5], &rotation[6],
                       &rotation[7], &rotation[8]) == 11) {
                use_nu_and_g = true;
                enough_parameters = true;
            }
        } else if (strcmp(sub_type, "_N1G1") == 0) {
            if (sscanf(parameters, "%lf,%lf", &nu, &G) == 2) {
                use_nu_and_g = true;
                enough_parameters = true;
            }
        } else if (strcmp(sub_type, "_N1R9") == 0) {
            if (sscanf(parameters, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &nu, &rotation[0], &rotation[1],
                       &rotation[2], &rotation[3], &rotation[4], &rotation[5], &rotation[6], &rotation[7],
                       &rotation[8]) == 10) {
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
            nu12 = nu * sqrt(E1 / E2);
            nu23 = nu * sqrt(E2 / E3);
            nu31 = nu * sqrt(E3 / E1);
            G12 = G * sqrt(E1 * E2) / (2.0 * (1.0 + nu));
            G23 = G * sqrt(E2 * E3) / (2.0 * (1.0 + nu));
            G31 = G * sqrt(E3 * E1) / (2.0 * (1.0 + nu));
        }
    }

    if (enough_parameters && (G12 > 0) && (G23 > 0) && (G31 > 0)) {
        return std::make_unique<OrthotropicMaterial>(material_name, density, E1, E2, E3, nu12, nu23, nu31, G12, G23,
                                                     G31, rotation);
    }

    printf("Error: incorrect material specification in file %s. Offending line: %s\n", filename.string().c_str(),
           offending_line.c_str());
    throw 14;
}

}  // namespace

std::unique_ptr<VolumetricMesh::Material> parse_ascii_material(const std::string& material_name,
                                                               const std::string& material_specification,
                                                               const std::filesystem::path& filename,
                                                               const std::string& offending_line) {
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
        double density = 0.0;
        double E = 0.0;
        double nu = 0.0;
        sscanf(parameters, "%lf,%lf,%lf", &density, &E, &nu);

        if ((E > 0) && (nu > -1.0) && (nu < 0.5) && (density > 0)) {
            return std::make_unique<ENuMaterial>(material_name, density, E, nu);
        }

        printf("Error: incorrect material specification in file %s. Offending line: %s\n",
               filename.string().c_str(), offending_line.c_str());
        throw 13;
    }

    if (strncmp(material_type, "ORTHOTROPIC", 11) == 0) {
        return parse_orthotropic_material(material_name, material_type, parameters, filename, offending_line);
    }

    if (strncmp(material_type, "MOONEYRIVLIN", 12) == 0) {
        double density = 0.0;
        double mu01 = 0.0;
        double mu10 = 0.0;
        double v1 = 0.0;
        sscanf(parameters, "%lf,%lf,%lf,%lf", &density, &mu01, &mu10, &v1);

        if (density > 0) {
            return std::make_unique<MooneyRivlinMaterial>(material_name, density, mu01, mu10, v1);
        }

        printf("Error: incorrect material specification in file %s. Offending line:\n%s\n",
               filename.string().c_str(), offending_line.c_str());
        throw 15;
    }

    printf("Error: incorrect material specification in file %s. Offending line:\n%s\n", filename.string().c_str(),
           offending_line.c_str());
    throw 16;
}

std::unique_ptr<VolumetricMesh::Material> read_binary_material(std::istream& input) {
    char material_name[4096];
    int length = 0;
    read_exact(input, &length, 1);
    read_exact(input, material_name, static_cast<size_t>(length));
    material_name[length] = '\0';

    int material_type = 0;
    read_exact(input, &material_type, 1);

    switch (material_type) {
        case static_cast<int>(MaterialType::ENu): {
            double material_property[ENU_NUM_PROPERTIES];
            read_exact(input, material_property, ENU_NUM_PROPERTIES);

            if ((material_property[ENU_E] > 0) && (material_property[ENU_NU] > -1.0) &&
                (material_property[ENU_NU] < 0.5) && (material_property[ENU_DENSITY] > 0)) {
                return std::make_unique<ENuMaterial>(material_name, material_property[ENU_DENSITY],
                                                     material_property[ENU_E], material_property[ENU_NU]);
            }

            printf("Error in io::load_binary_data: incorrect material specification in file stream.\n");
            throw 7;
        }

        case static_cast<int>(MaterialType::Orthotropic): {
            double material_property[ORTHOTROPIC_NUM_PROPERTIES];
            double rotation[9];
            read_exact(input, material_property, ORTHOTROPIC_NUM_PROPERTIES);
            read_exact(input, rotation, 9);

            if ((material_property[ORTHOTROPIC_E1] > 0) && (material_property[ORTHOTROPIC_E2] > 0) &&
                (material_property[ORTHOTROPIC_E3] > 0) && (material_property[ORTHOTROPIC_G12] > 0) &&
                (material_property[ORTHOTROPIC_G23] > 0) && (material_property[ORTHOTROPIC_G31] > 0) &&
                (material_property[ORTHOTROPIC_DENSITY] > 0)) {
                return std::make_unique<OrthotropicMaterial>(
                    material_name, material_property[ORTHOTROPIC_DENSITY], material_property[ORTHOTROPIC_E1],
                    material_property[ORTHOTROPIC_E2], material_property[ORTHOTROPIC_E3],
                    material_property[ORTHOTROPIC_NU12], material_property[ORTHOTROPIC_NU23],
                    material_property[ORTHOTROPIC_NU31], material_property[ORTHOTROPIC_G12],
                    material_property[ORTHOTROPIC_G23], material_property[ORTHOTROPIC_G31], rotation);
            }

            printf("Error in io::load_binary_data: incorrect orthotropic material specification.\n");
            throw 14;
        }

        case static_cast<int>(MaterialType::MooneyRivlin): {
            double material_property[MOONEYRIVLIN_NUM_PROPERTIES];
            read_exact(input, material_property, MOONEYRIVLIN_NUM_PROPERTIES);
            if (material_property[MOONEYRIVLIN_DENSITY] > 0) {
                return std::make_unique<MooneyRivlinMaterial>(material_name, material_property[MOONEYRIVLIN_DENSITY],
                                                              material_property[MOONEYRIVLIN_MU01],
                                                              material_property[MOONEYRIVLIN_MU10],
                                                              material_property[MOONEYRIVLIN_V1]);
            }

            printf("Error in io::load_binary_data: incorrect Mooney-Rivlin material specification.\n");
            throw 8;
        }

        default:
            printf("Error in io::load_binary_data: material type %d is unknown.\n", material_type);
            throw 9;
    }
}

void write_ascii_material(FILE* output, const VolumetricMesh::Material& material) {
    fprintf(output, "*MATERIAL %s\n", material.getName().c_str());

    if (material.getType() == MaterialType::ENu) {
        const auto* enu = downcastENuMaterial(&material);
        fprintf(output, "ENU, %.15G, %.15G, %.15G\n", enu->getDensity(), enu->getE(), enu->getNu());
    } else if (material.getType() == MaterialType::Orthotropic) {
        const auto* orthotropic = downcastOrthotropicMaterial(&material);
        double rotation[9];
        orthotropic->getR(rotation);
        fprintf(output,
                "ORTHOTROPIC, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, "
                "%.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G\n",
                orthotropic->getDensity(), orthotropic->getE1(), orthotropic->getE2(), orthotropic->getE3(),
                orthotropic->getNu12(), orthotropic->getNu23(), orthotropic->getNu31(), orthotropic->getG12(),
                orthotropic->getG23(), orthotropic->getG31(), rotation[0], rotation[1], rotation[2], rotation[3],
                rotation[4], rotation[5], rotation[6], rotation[7], rotation[8]);
    } else if (material.getType() == MaterialType::MooneyRivlin) {
        const auto* mooney = downcastMooneyRivlinMaterial(&material);
        fprintf(output, "MOONEYRIVLIN, %.15G, %.15G, %.20G, %.15G\n", mooney->getDensity(), mooney->getmu01(),
                mooney->getmu10(), mooney->getv1());
    }

    fprintf(output, "\n");
}

unsigned int write_binary_material(std::ostream& output, const VolumetricMesh::Material& material) {
    unsigned int bytes_written = 0;

    auto count_write = [&](const auto* data, size_t count) {
        using T = std::remove_pointer_t<decltype(data)>;
        write_exact(output, data, count);
        bytes_written += static_cast<unsigned int>(sizeof(T) * count);
    };

    const unsigned int length = static_cast<unsigned int>(material.getName().size());
    count_write(&length, 1);
    output.write(material.getName().c_str(), static_cast<std::streamsize>(length));
    if (!output) {
        throw std::runtime_error("Failed to write material name.");
    }
    bytes_written += length;

    const int material_type = static_cast<int>(material.getType());
    count_write(&material_type, 1);

    if (material.getType() == MaterialType::ENu) {
        const auto* enu = downcastENuMaterial(&material);
        const double properties[] = {enu->getDensity(), enu->getE(), enu->getNu()};
        count_write(properties, 3);
    } else if (material.getType() == MaterialType::MooneyRivlin) {
        const auto* mooney = downcastMooneyRivlinMaterial(&material);
        const double properties[] = {mooney->getDensity(), mooney->getmu01(), mooney->getmu10(), mooney->getv1()};
        count_write(properties, 4);
    } else if (material.getType() == MaterialType::Orthotropic) {
        const auto* orthotropic = downcastOrthotropicMaterial(&material);
        const double properties[] = {orthotropic->getDensity(), orthotropic->getE1(), orthotropic->getE2(),
                                     orthotropic->getE3(), orthotropic->getNu12(), orthotropic->getNu23(),
                                     orthotropic->getNu31(), orthotropic->getG12(), orthotropic->getG23(),
                                     orthotropic->getG31()};
        count_write(properties, 10);
        double rotation[9];
        orthotropic->getR(rotation);
        count_write(rotation, 9);
    } else {
        throw std::runtime_error("Unknown material type.");
    }

    return bytes_written;
}

}  // namespace pgo::VolumetricMeshes::io::detail
