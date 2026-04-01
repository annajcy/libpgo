#include "io/mesh_save.h"
#include "tetMesh.h"

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: mshFileToVegFile <input.msh> <output.veg>\n";
        return 1;
    }

    const char* inputMshFile  = argv[1];
    const char* outputVegFile = argv[2];

    pgo::VolumetricMeshes::TetMesh tetMesh(inputMshFile, 1, 1);

    if (pgo::VolumetricMeshes::io::save(tetMesh, outputVegFile) != 0) {
        std::cerr << "Failed to write VEG file: " << outputVegFile << "\n";
        return 1;
    }

    std::cout << "Successfully converted " << inputMshFile << " to " << outputVegFile << "\n";
    return 0;
}
