#include "volumetricMeshIO.h"

#include "cubicMesh.h"
#include "tetMesh.h"
#include "volumetricMeshENuMaterial.h"
#include "volumetricMeshMooneyRivlinMaterial.h"
#include "volumetricMeshOrthotropicMaterial.h"
#include "volumetricMeshParser.h"

#include "stringHelper.h"

#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace pgo::VolumetricMeshes {
namespace io {
namespace {

using LoadedMeshData = detail::LoadedMeshData;
using ElementType = VolumetricMesh::ElementType;
using FileFormatType = VolumetricMesh::FileFormatType;
using MaterialType = VolumetricMesh::Material::MaterialType;
using ENuMaterial = VolumetricMesh::ENuMaterial;
using OrthotropicMaterial = VolumetricMesh::OrthotropicMaterial;
using MooneyRivlinMaterial = VolumetricMesh::MooneyRivlinMaterial;

constexpr ElementType INVALID = ElementType::Invalid;
constexpr ElementType TET = ElementType::Tet;
constexpr ElementType CUBIC = ElementType::Cubic;
constexpr FileFormatType ASCII = FileFormatType::Ascii;
constexpr FileFormatType BINARY = FileFormatType::Binary;
constexpr FileFormatType BY_EXT = FileFormatType::ByExtension;
constexpr FileFormatType UNKNOWN = FileFormatType::Unknown;

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
    if (data.elementType != expectedType) {
        printf("Error: mesh is not a %s mesh.\n", expectedType == TET ? "tet" : "cubic");
        throw 11;
    }
    const int expectedVerticesPerElement = vertices_per_element(expectedType);
    if (data.numElementVertices != expectedVerticesPerElement) {
        printf("Error: mesh data has %d vertices per element; expected %d.\n", data.numElementVertices,
               expectedVerticesPerElement);
        throw 12;
    }
}

LoadedMeshData load_ascii_data(const std::filesystem::path& filename, int verbose) {
    const std::string filenameString = filename.string();
    const char*       filenameCStr   = filenameString.c_str();

    VolumetricMeshParser parser;
    if (parser.open(filenameCStr) != 0) {
        printf("Error: could not open file %s.\n", filenameCStr);
        throw 1;
    }

    LoadedMeshData data;

    int countNumVertices = 0;
    int countNumElements = 0;
    int numVertices = -1;
    int numElements = -1;
    int numMaterials = 0;
    int numSets = 1;
    int numRegions = 0;
    int parseState = 0;
    int oneIndexedVertices = 1;
    int oneIndexedElements = 1;
    char lineBuffer[1024];

    while (parser.getNextLine(lineBuffer, 0, 0) != nullptr) {
        if ((parseState == 0) && (strncmp(lineBuffer, "*VERTICES", 9) == 0)) {
            parseState = 1;

            if (parser.getNextLine(lineBuffer, 0, 0) != nullptr) {
                sscanf(lineBuffer, "%d", &numVertices);
                data.vertices.resize(static_cast<size_t>(numVertices));
            } else {
                printf("Error: file %s is not in the .veg format. Offending line:\n%s\n", filenameCStr, lineBuffer);
                throw 2;
            }
            continue;
        }

        if ((parseState == 1) && (strncmp(lineBuffer, "*ELEMENTS", 9) == 0)) {
            parseState = 2;

            if (parser.getNextLine(lineBuffer) != nullptr) {
                parser.removeWhitespace(lineBuffer);

                if (strncmp(lineBuffer, "TET", 3) == 0)
                    data.elementType = TET;
                else if (strncmp(lineBuffer, "CUBIC", 5) == 0)
                    data.elementType = CUBIC;
                else {
                    printf("Error: unknown mesh type %s in file %s\n", lineBuffer, filenameCStr);
                    throw 3;
                }

                data.numElementVertices = vertices_per_element(data.elementType);
            } else {
                printf("Error: file %s is not in the .veg format. Offending line:\n%s\n", filenameCStr, lineBuffer);
                throw 4;
            }

            if (parser.getNextLine(lineBuffer, 0, 0) != nullptr) {
                sscanf(lineBuffer, "%d", &numElements);
                data.elements.resize(static_cast<size_t>(numElements * data.numElementVertices));
            } else {
                printf("Error: file %s is not in the .veg format. Offending line:\n%s\n", filenameCStr, lineBuffer);
                throw 5;
            }
            continue;
        }

        if ((parseState == 2) && (lineBuffer[0] == '*'))
            parseState = 3;

        if (parseState == 1) {
            if (countNumVertices >= numVertices) {
                printf("Error: mismatch in the number of vertices in %s.\n", filenameCStr);
                throw 6;
            }

            char* ch = lineBuffer;
            while ((*ch == ' ') || (*ch == ',') || (*ch == '\t'))
                ch++;

            int index = -1;
            sscanf(ch, "%d", &index);
            while ((*ch != ' ') && (*ch != ',') && (*ch != '\t') && (*ch != 0))
                ch++;

            if (index == 0)
                oneIndexedVertices = 0;

            double pos[3];
            for (int i = 0; i < 3; i++) {
                while ((*ch == ' ') || (*ch == ',') || (*ch == '\t'))
                    ch++;

                if (*ch == 0) {
                    printf("Error parsing line %s in file %s.\n", lineBuffer, filenameCStr);
                    throw 7;
                }

                sscanf(ch, "%lf", &pos[i]);
                while ((*ch != ' ') && (*ch != ',') && (*ch != '\t') && (*ch != 0))
                    ch++;
            }

            data.vertices[static_cast<size_t>(countNumVertices)] = Vec3d(pos);
            countNumVertices++;
        }

        if (parseState == 2) {
            if (countNumElements >= numElements) {
                printf("Error: mismatch in the number of elements in %s.\n", filenameCStr);
                throw 8;
            }

            char* ch = lineBuffer;
            while ((*ch == ' ') || (*ch == ',') || (*ch == '\t'))
                ch++;

            int index = -1;
            sscanf(ch, "%d", &index);
            if (index == 0)
                oneIndexedElements = 0;

            while ((*ch != ' ') && (*ch != ',') && (*ch != '\t') && (*ch != 0))
                ch++;

            for (int i = 0; i < data.numElementVertices; i++) {
                while ((*ch == ' ') || (*ch == ',') || (*ch == '\t'))
                    ch++;

                if (*ch == 0) {
                    printf("Error parsing line %s in file %s.\n", lineBuffer, filenameCStr);
                    throw 9;
                }

                int vertexIndex = -1;
                sscanf(ch, "%d", &vertexIndex);
                data.elements[static_cast<size_t>(countNumElements * data.numElementVertices + i)] =
                    vertexIndex - oneIndexedVertices;

                while ((*ch != ' ') && (*ch != ',') && (*ch != '\t') && (*ch != 0))
                    ch++;
            }

            countNumElements++;
        }

        if (strncmp(lineBuffer, "*MATERIAL", 9) == 0)
            numMaterials++;
        if (strncmp(lineBuffer, "*SET", 4) == 0)
            numSets++;
        if (strncmp(lineBuffer, "*REGION", 7) == 0)
            numRegions++;
    }

    if (numElements < 0) {
        printf("Error: incorrect number of elements. File %s may not be in the .veg format.\n", filenameCStr);
        throw 10;
    }

    parser.rewindToStart();

    if (verbose) {
        if (numMaterials == 0)
            printf("Warning: no materials encountered in %s.\n", filenameCStr);
        if (numRegions == 0)
            printf("Warning: no regions encountered in %s.\n", filenameCStr);
    }

    data.materials.resize(static_cast<size_t>(numMaterials));
    data.sets.resize(static_cast<size_t>(numSets));
    data.regions.resize(static_cast<size_t>(numRegions));
    data.sets[0] = VolumetricMesh::generateAllElementsSet(numElements);

    int countNumMaterials = 0;
    int countNumSets = 1;
    int countNumRegions = 0;

    std::map<std::string, int> materialMap;
    std::map<std::string, int> setMap;
    setMap.insert(std::pair<std::string, int>(data.sets[0].getName(), 0));

    parseState = 0;

    while (parser.getNextLine(lineBuffer, 0, 0) != nullptr) {
        if ((parseState == 11) && (lineBuffer[0] == '*'))
            parseState = 0;

        if ((parseState == 0) && (strncmp(lineBuffer, "*MATERIAL", 9) == 0)) {
            parser.removeWhitespace(lineBuffer);

            char materialNameC[4096];
            strcpy(materialNameC, &lineBuffer[9]);

            char materialSpecification[4096];
            if (parser.getNextLine(lineBuffer) != nullptr) {
                parser.removeWhitespace(lineBuffer);
                sscanf(lineBuffer, "%s", materialSpecification);
            } else {
                printf("Error: incorrect material in file %s. Offending line:\n%s\n", filenameCStr, lineBuffer);
                throw 11;
            }

            char* ch = materialSpecification;
            while ((*ch != ',') && (*ch != 0))
                ch++;

            if (*ch == 0) {
                printf("Error parsing file %s. Offending line: %s.\n", filenameCStr, lineBuffer);
                throw 12;
            }

            char materialType[4096];
            unsigned int materialTypeLength = static_cast<unsigned int>(ch - materialSpecification);
            memcpy(materialType, materialSpecification, sizeof(unsigned char) * materialTypeLength);
            *(materialType + materialTypeLength) = 0;

            ch++;

            if (strcmp(materialType, "ENU") == 0) {
                double density, E, nu;
                sscanf(ch, "%lf,%lf,%lf", &density, &E, &nu);

                if ((E > 0) && (nu > -1.0) && (nu < 0.5) && (density > 0)) {
                    std::string name(materialNameC);
                    data.materials[static_cast<size_t>(countNumMaterials)] =
                        std::make_unique<ENuMaterial>(name, density, E, nu);
                    materialMap.insert(std::pair<std::string, int>(name, countNumMaterials));
                } else {
                    printf("Error: incorrect material specification in file %s. Offending line: %s\n", filenameCStr,
                           lineBuffer);
                    throw 13;
                }
            } else if (strncmp(materialType, "ORTHOTROPIC", 11) == 0) {
                double density = 0.0, E1 = 0.0, E2 = 0.0, E3 = 0.0, nu12 = 0.0, nu23 = 0.0, nu31 = 0.0, G12 = 0.0,
                       G23 = 0.0, G31 = 0.0;
                double nu = 0.0, G = 1.0;
                bool useNuAndG = false;
                double R[9];
                memset(R, 0, sizeof(R));
                R[0] = R[4] = R[8] = 1.0;

                char* subType = materialType + 11;
                bool enoughParameters = false;

                if ((sscanf(ch, "%lf,%lf,%lf,%lf", &density, &E1, &E2, &E3) == 4) &&
                    ((E1 > 0) && (E2 > 0) && (E3 > 0) && (density > 0))) {
                    for (int i = 0; i < 4; i++) {
                        while ((*ch != ',') && (*ch != 0))
                            ch++;
                        if (*ch == 0)
                            break;
                        ch++;
                    }

                    if ((*subType == 0) || (strcmp(subType, "_N3G3R9") == 0)) {
                        if (sscanf(ch, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &nu12, &nu23,
                                   &nu31, &G12, &G23, &G31, &R[0], &R[1], &R[2], &R[3], &R[4], &R[5], &R[6], &R[7],
                                   &R[8]) == 15)
                            enoughParameters = true;
                    } else if (strcmp(subType, "_N3G3") == 0) {
                        if (sscanf(ch, "%lf,%lf,%lf,%lf,%lf,%lf", &nu12, &nu23, &nu31, &G12, &G23, &G31) == 6)
                            enoughParameters = true;
                    } else if (strcmp(subType, "_N1G1R9") == 0) {
                        if (sscanf(ch, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &nu, &G, &R[0], &R[1], &R[2],
                                   &R[3], &R[4], &R[5], &R[6], &R[7], &R[8]) == 11) {
                            useNuAndG = true;
                            enoughParameters = true;
                        }
                    } else if (strcmp(subType, "_N1G1") == 0) {
                        if (sscanf(ch, "%lf,%lf", &nu, &G) == 2) {
                            useNuAndG = true;
                            enoughParameters = true;
                        }
                    } else if (strcmp(subType, "_N1R9") == 0) {
                        if (sscanf(ch, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &nu, &R[0], &R[1], &R[2], &R[3],
                                   &R[4], &R[5], &R[6], &R[7], &R[8]) == 10) {
                            useNuAndG = true;
                            enoughParameters = true;
                        }
                    } else if (strcmp(subType, "_N1") == 0) {
                        if (sscanf(ch, "%lf", &nu) == 1) {
                            useNuAndG = true;
                            enoughParameters = true;
                        }
                    } else {
                        printf("Error: incorrect orthortropic material type \"%s\" in file %s. Offending line: %s\n",
                               subType, filenameCStr, lineBuffer);
                        throw 14;
                    }

                    if (useNuAndG && (nu > -1.0) && (nu < 0.5)) {
                        nu12 = nu * sqrt(E1 / E2);
                        nu23 = nu * sqrt(E2 / E3);
                        nu31 = nu * sqrt(E3 / E1);
                        G12 = G * sqrt(E1 * E2) / (2.0 * (1.0 + nu));
                        G23 = G * sqrt(E2 * E3) / (2.0 * (1.0 + nu));
                        G31 = G * sqrt(E3 * E1) / (2.0 * (1.0 + nu));
                    }
                }

                if (enoughParameters && (G12 > 0) && (G23 > 0) && (G31 > 0)) {
                    std::string name(materialNameC);
                    data.materials[static_cast<size_t>(countNumMaterials)] =
                        std::make_unique<OrthotropicMaterial>(name, density, E1, E2, E3, nu12, nu23, nu31, G12, G23,
                                                              G31, R);
                    materialMap.insert(std::pair<std::string, int>(name, countNumMaterials));
                } else {
                    printf("Error: incorrect material specification in file %s. Offending line: %s\n", filenameCStr,
                           lineBuffer);
                    throw 14;
                }
            } else if (strncmp(materialType, "MOONEYRIVLIN", 12) == 0) {
                double density, mu01, mu10, v1;
                sscanf(ch, "%lf,%lf,%lf,%lf", &density, &mu01, &mu10, &v1);

                if (density > 0) {
                    std::string name(materialNameC);
                    data.materials[static_cast<size_t>(countNumMaterials)] =
                        std::make_unique<MooneyRivlinMaterial>(name, density, mu01, mu10, v1);
                    materialMap.insert(std::pair<std::string, int>(name, countNumMaterials));
                } else {
                    printf("Error: incorrect material specification in file %s. Offending line:\n%s\n", filenameCStr,
                           lineBuffer);
                    throw 15;
                }
            } else {
                printf("Error: incorrect material specification in file %s. Offending line:\n%s\n", filenameCStr,
                       lineBuffer);
                throw 16;
            }

            countNumMaterials++;
        }

        if ((parseState == 0) && (strncmp(lineBuffer, "*REGION", 7) == 0)) {
            parser.removeWhitespace(lineBuffer);

            char setNameC[4096];
            char materialNameC[4096];

            if (parser.getNextLine(lineBuffer) != nullptr) {
                parser.removeWhitespace(lineBuffer);

                char* ch = lineBuffer;
                while ((*ch != ',') && (*ch != 0))
                    ch++;

                if (*ch == 0) {
                    printf("Error parsing file %s. Offending line: %s.\n", filenameCStr, lineBuffer);
                    throw 17;
                }

                *ch = 0;
                strcpy(setNameC, lineBuffer);
                *ch = ',';
                ch++;
                strcpy(materialNameC, ch);
            } else {
                printf("Error: file %s is not in the .veg format. Offending line:\n%s\n", filenameCStr, lineBuffer);
                throw 18;
            }

            int setNum = -1;
            auto it = setMap.find(std::string(setNameC));
            if (it != setMap.end()) {
                setNum = it->second;
            } else {
                printf("Error: set \"%s\" not found among the sets.\n", setNameC);
                printf("All existing sets:\n");
                for (const auto& p : setMap)
                    printf("%i: \"%s\"\n", p.second, p.first.c_str());
                printf("\n");
                throw 19;
            }

            int materialNum = -1;
            it = materialMap.find(std::string(materialNameC));
            if (it != materialMap.end()) {
                materialNum = it->second;
            } else {
                printf("Error: material %s not found among the materials.\n", materialNameC);
                throw 20;
            }

            data.regions[static_cast<size_t>(countNumRegions)] = VolumetricMesh::Region(materialNum, setNum);
            countNumRegions++;
        }

        if (parseState == 11) {
            parser.removeWhitespace(lineBuffer);

            char* pch = strtok(lineBuffer, ",");
            while ((pch != nullptr) && (isdigit(*pch))) {
                int newElement = atoi(pch);
                int ind = newElement - oneIndexedElements;
                if (ind >= numElements || ind < 0) {
                    printf("Error: set element index: %d out of bounds.\n", newElement);
                    throw 21;
                }
                data.sets[static_cast<size_t>(countNumSets - 1)].insert(ind);
                pch = strtok(nullptr, ",");
            }
        }

        if ((parseState == 0) && (strncmp(lineBuffer, "*SET", 4) == 0)) {
            parser.removeWhitespace(lineBuffer);

            std::string name(BasicAlgorithms::stripLight(&lineBuffer[4]));
            data.sets[static_cast<size_t>(countNumSets)] = VolumetricMesh::Set(name);
            setMap.insert(std::pair<std::string, int>(name, countNumSets));
            countNumSets++;
            parseState = 11;
        }
    }

    parser.close();
    return data;
}

LoadedMeshData load_binary_data(std::istream& binaryInputStream) {
    LoadedMeshData data;

    double version = 0.0;
    read_exact(binaryInputStream, &version, 1);

    int eleType = 0;
    read_exact(binaryInputStream, &eleType, 1);
    data.elementType = element_type_from_int(eleType);
    if (data.elementType == INVALID) {
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

    read_exact(binaryInputStream, &data.numElementVertices, 1);
    if (data.numElementVertices <= 0) {
        printf("Error in io::load_binary_data: incorrect number of vertices per element.\n");
        throw 5;
    }

    data.elements.resize(static_cast<size_t>(numElements * data.numElementVertices));
    read_exact(binaryInputStream, data.elements.data(), data.elements.size());

    int numMaterials = 0;
    read_exact(binaryInputStream, &numMaterials, 1);
    if (numMaterials < 0) {
        printf("Error in io::load_binary_data: incorrect number of materials.\n");
        throw 6;
    }

    data.materials.resize(static_cast<size_t>(numMaterials));
    for (int materialIndex = 0; materialIndex < numMaterials; materialIndex++) {
        char materialName[4096];
        int length = 0;
        read_exact(binaryInputStream, &length, 1);
        read_exact(binaryInputStream, materialName, static_cast<size_t>(length));
        materialName[length] = '\0';

        int matType = 0;
        read_exact(binaryInputStream, &matType, 1);

        switch (matType) {
            case static_cast<int>(MaterialType::ENu): {
                double materialProperty[ENU_NUM_PROPERTIES];
                read_exact(binaryInputStream, materialProperty, ENU_NUM_PROPERTIES);

                if ((materialProperty[ENU_E] > 0) && (materialProperty[ENU_NU] > -1.0) &&
                    (materialProperty[ENU_NU] < 0.5) && (materialProperty[ENU_DENSITY] > 0)) {
                    data.materials[static_cast<size_t>(materialIndex)] =
                        std::make_unique<ENuMaterial>(materialName, materialProperty[ENU_DENSITY],
                                                      materialProperty[ENU_E], materialProperty[ENU_NU]);
                } else {
                    printf("Error in io::load_binary_data: incorrect material specification in file stream.\n");
                    throw 7;
                }
            } break;

            case static_cast<int>(MaterialType::Orthotropic): {
                double materialProperty[ORTHOTROPIC_NUM_PROPERTIES];
                double R[9];
                read_exact(binaryInputStream, materialProperty, ORTHOTROPIC_NUM_PROPERTIES);
                read_exact(binaryInputStream, R, 9);

                if ((materialProperty[ORTHOTROPIC_E1] > 0) && (materialProperty[ORTHOTROPIC_E2] > 0) &&
                    (materialProperty[ORTHOTROPIC_E3] > 0) && (materialProperty[ORTHOTROPIC_G12] > 0) &&
                    (materialProperty[ORTHOTROPIC_G23] > 0) && (materialProperty[ORTHOTROPIC_G31] > 0) &&
                    (materialProperty[ORTHOTROPIC_DENSITY] > 0)) {
                    data.materials[static_cast<size_t>(materialIndex)] = std::make_unique<OrthotropicMaterial>(
                        materialName, materialProperty[ORTHOTROPIC_DENSITY], materialProperty[ORTHOTROPIC_E1],
                        materialProperty[ORTHOTROPIC_E2], materialProperty[ORTHOTROPIC_E3],
                        materialProperty[ORTHOTROPIC_NU12], materialProperty[ORTHOTROPIC_NU23],
                        materialProperty[ORTHOTROPIC_NU31], materialProperty[ORTHOTROPIC_G12],
                        materialProperty[ORTHOTROPIC_G23], materialProperty[ORTHOTROPIC_G31], R);
                } else {
                    printf("Error in io::load_binary_data: incorrect orthotropic material specification.\n");
                    throw 14;
                }
            } break;

            case static_cast<int>(MaterialType::MooneyRivlin): {
                double materialProperty[MOONEYRIVLIN_NUM_PROPERTIES];
                read_exact(binaryInputStream, materialProperty, MOONEYRIVLIN_NUM_PROPERTIES);
                if (materialProperty[MOONEYRIVLIN_DENSITY] > 0) {
                    data.materials[static_cast<size_t>(materialIndex)] = std::make_unique<MooneyRivlinMaterial>(
                        materialName, materialProperty[MOONEYRIVLIN_DENSITY], materialProperty[MOONEYRIVLIN_MU01],
                        materialProperty[MOONEYRIVLIN_MU10], materialProperty[MOONEYRIVLIN_V1]);
                } else {
                    printf("Error in io::load_binary_data: incorrect Mooney-Rivlin material specification.\n");
                    throw 8;
                }
            } break;

            default:
                printf("Error in io::load_binary_data: material type %d is unknown.\n", matType);
                throw 9;
        }
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

    return data;
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
            return load_ascii_data(filename, verbose);
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

void save_ascii_impl(const VolumetricMesh& mesh, const std::filesystem::path& filename) {
    FILE* fout = fopen(filename.string().c_str(), "w");
    if (!fout) {
        printf("Error: could not write to %s.\n", filename.string().c_str());
        throw std::runtime_error("Failed to open ascii mesh file.");
    }

    fprintf(fout, "# Vega mesh file.\n");
    fprintf(fout, "# %d vertices, %d elements\n\n", mesh.getNumVertices(), mesh.getNumElements());

    fprintf(fout, "*VERTICES\n");
    fprintf(fout, "%d 3 0 0\n", mesh.getNumVertices());
    for (int i = 0; i < mesh.getNumVertices(); i++) {
        const Vec3d& v = mesh.getVertex(i);
        fprintf(fout, "%d %.15G %.15G %.15G\n", i + 1, v[0], v[1], v[2]);
    }
    fprintf(fout, "\n");

    fprintf(fout, "*ELEMENTS\n");
    fprintf(fout, "%s\n", mesh.getElementType() == TET ? "TET" : "CUBIC");
    fprintf(fout, "%d %d 0\n", mesh.getNumElements(), mesh.getNumElementVertices());
    for (int el = 0; el < mesh.getNumElements(); el++) {
        fprintf(fout, "%d ", el + 1);
        for (int j = 0; j < mesh.getNumElementVertices(); j++) {
            fprintf(fout, "%d", mesh.getVertexIndex(el, j) + 1);
            if (j != mesh.getNumElementVertices() - 1)
                fprintf(fout, " ");
        }
        fprintf(fout, "\n");
    }
    fprintf(fout, "\n");

    for (int materialIndex = 0; materialIndex < mesh.getNumMaterials(); materialIndex++) {
        const auto* material = mesh.getMaterial(materialIndex);
        fprintf(fout, "*MATERIAL %s\n", material->getName().c_str());

        if (material->getType() == MaterialType::ENu) {
            const auto* enu = downcastENuMaterial(material);
            fprintf(fout, "ENU, %.15G, %.15G, %.15G\n", enu->getDensity(), enu->getE(), enu->getNu());
        } else if (material->getType() == MaterialType::Orthotropic) {
            const auto* ortho = downcastOrthotropicMaterial(material);
            double R[9];
            ortho->getR(R);
            fprintf(fout,
                    "ORTHOTROPIC, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G, "
                    "%.15G, %.15G, %.15G, %.15G, %.15G, %.15G, %.15G\n",
                    ortho->getDensity(), ortho->getE1(), ortho->getE2(), ortho->getE3(), ortho->getNu12(),
                    ortho->getNu23(), ortho->getNu31(), ortho->getG12(), ortho->getG23(), ortho->getG31(), R[0], R[1],
                    R[2], R[3], R[4], R[5], R[6], R[7], R[8]);
        } else if (material->getType() == MaterialType::MooneyRivlin) {
            const auto* mooney = downcastMooneyRivlinMaterial(material);
            fprintf(fout, "MOONEYRIVLIN, %.15G, %.15G, %.20G %.15G\n", mooney->getDensity(), mooney->getmu01(),
                    mooney->getmu10(), mooney->getv1());
        }
        fprintf(fout, "\n");
    }

    for (int setIndex = 1; setIndex < mesh.getNumSets(); setIndex++) {
        const auto& set = mesh.getSet(setIndex);
        fprintf(fout, "*SET %s\n", set.getName().c_str());
        int count = 0;
        for (int el : set.getElements()) {
            fprintf(fout, "%d, ", el + 1);
            count++;
            if (count == 8) {
                fprintf(fout, "\n");
                count = 0;
            }
        }
        if (count != 0)
            fprintf(fout, "\n");
        fprintf(fout, "\n");
    }

    for (int regionIndex = 0; regionIndex < mesh.getNumRegions(); regionIndex++) {
        const auto& region = mesh.getRegion(regionIndex);
        fprintf(fout, "*REGION\n");
        fprintf(fout, "%s, %s\n", mesh.getSet(region.getSetIndex()).getName().c_str(),
                mesh.getMaterial(region.getMaterialIndex())->getName().c_str());
        fprintf(fout, "\n");
    }

    fclose(fout);
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
        const auto* material = mesh.getMaterial(materialIndex);
        const unsigned int length = static_cast<unsigned int>(material->getName().size());
        count_write(&length, 1);
        out.write(material->getName().c_str(), static_cast<std::streamsize>(length));
        if (!out)
            throw std::runtime_error("Failed to write material name.");
        totalBytesWritten += length;

        const int matType = static_cast<int>(material->getType());
        count_write(&matType, 1);

        if (material->getType() == MaterialType::ENu) {
            const auto* enu = downcastENuMaterial(material);
            const double properties[] = {enu->getDensity(), enu->getE(), enu->getNu()};
            count_write(properties, 3);
        } else if (material->getType() == MaterialType::MooneyRivlin) {
            const auto* mooney = downcastMooneyRivlinMaterial(material);
            const double properties[] = {mooney->getDensity(), mooney->getmu01(), mooney->getmu10(), mooney->getv1()};
            count_write(properties, 4);
        } else if (material->getType() == MaterialType::Orthotropic) {
            const auto* ortho = downcastOrthotropicMaterial(material);
            const double properties[] = {ortho->getDensity(), ortho->getE1(), ortho->getE2(), ortho->getE3(),
                                         ortho->getNu12(), ortho->getNu23(), ortho->getNu31(), ortho->getG12(),
                                         ortho->getG23(), ortho->getG31()};
            count_write(properties, 10);
            double R[9];
            ortho->getR(R);
            count_write(R, 9);
        } else {
            throw std::runtime_error("Unknown material type.");
        }
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

ElementType detect_ascii_element_type(const std::filesystem::path& filename) {
    VolumetricMeshParser parser;
    if (parser.open(filename.string().c_str()) != 0) {
        printf("Error: could not open file %s.\n", filename.string().c_str());
        return INVALID;
    }

    char lineBuffer[1024];
    ElementType result = INVALID;
    while (parser.getNextLine(lineBuffer, 0, 0) != nullptr) {
        if (strncmp(lineBuffer, "*ELEMENTS", 9) == 0) {
            if (parser.getNextLine(lineBuffer) != nullptr) {
                parser.removeWhitespace(lineBuffer);
                if (strncmp(lineBuffer, "TET", 3) == 0)
                    result = TET;
                else if (strncmp(lineBuffer, "CUBIC", 5) == 0)
                    result = CUBIC;
            }
            break;
        }
    }
    parser.close();
    return result;
}

ElementType detect_binary_element_type(std::istream& in) {
    double version = 0.0;
    int eleType = 0;
    read_exact(in, &version, 1);
    read_exact(in, &eleType, 1);
    return element_type_from_int(eleType);
}

}  // namespace

namespace detail {

FileFormatType detect_file_format_by_ext(const std::filesystem::path& filename) {
    const std::string filenameString = filename.string();
    if (BasicAlgorithms::iendWith(filenameString.c_str(), ".vegb"))
        return FileFormatType::Binary;
    if (BasicAlgorithms::iendWith(filenameString.c_str(), ".veg"))
        return FileFormatType::Ascii;
    return FileFormatType::Unknown;
}

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
        save_ascii_impl(mesh, filename);
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

VolumetricMesh::ElementType detect_element_type(const std::filesystem::path& filename,
                                                VolumetricMesh::FileFormatType fileFormat) {
    if (fileFormat == BY_EXT) {
        fileFormat = detect_file_format(filename);
        if (fileFormat == UNKNOWN) {
            printf("Unknown file extension when loading %s, try ASCII format...\n", filename.string().c_str());
            fileFormat = ASCII;
        }
    }

    if (fileFormat == ASCII)
        return detect_ascii_element_type(filename);

    if (fileFormat == BINARY) {
        std::ifstream in(filename, std::ios::binary);
        if (!in) {
            printf("Error in io::detect_element_type: could not open file %s.\n", filename.string().c_str());
            return INVALID;
        }
        return detect_binary_element_type(in);
    }

    printf("Error: the file format %d is unknown.\n", static_cast<int>(fileFormat));
    return INVALID;
}

VolumetricMesh::ElementType detect_element_type(std::span<const std::byte> binaryData) {
    if (binaryData.empty())
        throw std::runtime_error("binary data buffer is empty");
    std::string buffer(reinterpret_cast<const char*>(binaryData.data()), binaryData.size());
    std::istringstream in(buffer, std::ios::binary);
    return detect_binary_element_type(in);
}

VolumetricMesh::FileFormatType detect_file_format(const std::filesystem::path& filename) {
    return detail::detect_file_format_by_ext(filename);
}

}  // namespace io
}  // namespace pgo::VolumetricMeshes
