#include "cubicMesh.h"
#include "cubicMesherIO.h"
#include "cubicMesherUtils.h"
#include "pgoLogging.h"
#include "triangleMeshVoxelizer.h"
#include "uniformCubicMeshGenerator.h"

#include <argparse/argparse.hpp>

#include <array>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {
std::optional<cubic_mesher::TriangleMeshClassifyMode> parseTriangleMeshClassifyMode(const std::string& value) {
    if (value == "center") {
        return cubic_mesher::TriangleMeshClassifyMode::Center;
    }

    if (value == "conservative") {
        return cubic_mesher::TriangleMeshClassifyMode::Conservative;
    }

    return std::nullopt;
}
}  // namespace

int main(int argc, char* argv[]) {
    argparse::ArgumentParser program("Cubic mesh generator");

    argparse::ArgumentParser uniformCmd("uniform");
    uniformCmd.add_description("Generate a fully filled uniform cubic mesh");
    uniformCmd.add_argument("-r", "--resolution")
        .help("Grid resolution (NxNxN)")
        .required()
        .scan<'i', int>()
        .metavar("N");
    uniformCmd.add_argument("-o", "--output-mesh").help("Output cubic mesh filename (.veg)").required().metavar("PATH");
    uniformCmd.add_argument("-s", "--output-surface")
        .help("Output surface mesh filename (.obj), optional")
        .metavar("PATH");
    uniformCmd.add_argument("--size").help("Cube edge length").default_value(1.0).scan<'g', double>();
    uniformCmd.add_argument("--offset")
        .help("Translation applied to the generated cube center (x y z)")
        .nargs(3)
        .default_value(std::vector<double>{0.0, 0.0, 0.0})
        .scan<'g', double>()
        .metavar("X Y Z");
    uniformCmd.add_argument("--E").help("Young's modulus").default_value(1e6).scan<'g', double>();
    uniformCmd.add_argument("--nu").help("Poisson's ratio").default_value(0.45).scan<'g', double>();
    uniformCmd.add_argument("--density").help("Material density").default_value(1000.0).scan<'g', double>();

    argparse::ArgumentParser triangleMeshCmd("triangle-mesh");
    triangleMeshCmd.add_description("Voxelize a triangle mesh into a cubic mesh");
    triangleMeshCmd.add_argument("-i", "--input-mesh")
        .help("Input triangle mesh filename (.obj)")
        .required()
        .metavar("PATH");
    triangleMeshCmd.add_argument("-r", "--resolution")
        .help("Grid resolution of the regularized cubic domain (NxNxN)")
        .required()
        .scan<'i', int>()
        .metavar("N");
    triangleMeshCmd.add_argument("-o", "--output-mesh")
        .help("Output cubic mesh filename (.veg)")
        .required()
        .metavar("PATH");
    triangleMeshCmd.add_argument("-s", "--output-surface")
        .help("Output surface mesh filename (.obj), optional")
        .metavar("PATH");
    triangleMeshCmd.add_argument("--padding-voxels")
        .help("Number of empty voxel layers reserved around the regularized mesh domain")
        .default_value(1)
        .scan<'i', int>()
        .metavar("K");
    triangleMeshCmd.add_argument("--scale")
        .help("Uniform scale applied to the voxelized mesh after world-space reconstruction")
        .default_value(1.0)
        .scan<'g', double>();
    triangleMeshCmd.add_argument("--offset")
        .help("Translation applied to the voxelized mesh after world-space reconstruction (x y z)")
        .nargs(3)
        .default_value(std::vector<double>{0.0, 0.0, 0.0})
        .scan<'g', double>()
        .metavar("X Y Z");
    triangleMeshCmd.add_argument("--classify-mode")
        .help("Voxel occupancy mode: center or conservative")
        .default_value(std::string("conservative"))
        .metavar("MODE");
    triangleMeshCmd.add_argument("--E").help("Young's modulus").default_value(1e6).scan<'g', double>();
    triangleMeshCmd.add_argument("--nu").help("Poisson's ratio").default_value(0.45).scan<'g', double>();
    triangleMeshCmd.add_argument("--density").help("Material density").default_value(1000.0).scan<'g', double>();

    program.add_subparser(uniformCmd);
    program.add_subparser(triangleMeshCmd);

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    pgo::Logging::init();

    if (program.is_subcommand_used(uniformCmd)) {
        const int         resolution = uniformCmd.get<int>("--resolution");
        const double      size       = uniformCmd.get<double>("--size");
        const auto        offset     = uniformCmd.get<std::vector<double>>("--offset");
        const double      E          = uniformCmd.get<double>("--E");
        const double      nu         = uniformCmd.get<double>("--nu");
        const double      density    = uniformCmd.get<double>("--density");
        const std::string outputMesh = uniformCmd.get<std::string>("--output-mesh");

        cubic_mesher::UniformCubicMeshOptions options;
        options.resolution = resolution;
        options.size       = size;
        options.offset     = {offset[0], offset[1], offset[2]};
        options.E          = E;
        options.nu         = nu;
        options.density    = density;

        std::unique_ptr<pgo::VolumetricMeshes::CubicMesh> cubicMesh = cubic_mesher::createUniformCubicMesh(options);
        if (cubicMesh == nullptr) {
            return 1;
        }

        if (cubic_mesher::saveAndValidateCubicMesh(*cubicMesh, outputMesh) == false) {
            return 1;
        }

        if (uniformCmd.is_used("--output-surface")) {
            const std::string outputSurface = uniformCmd.get<std::string>("--output-surface");
            if (cubic_mesher::writeSurfaceMesh(*cubicMesh, outputSurface) == false) {
                return 1;
            }
        }

        SPDLOG_LOGGER_INFO(pgo::Logging::lgr(),
                           "Generated cubic mesh: N={}, size={}, offset=({}, {}, {}), vertices={}, elements={} -> {}",
                           resolution, size, offset[0], offset[1], offset[2], cubicMesh->getNumVertices(),
                           cubicMesh->getNumElements(), outputMesh);

        return 0;
    }

    if (program.is_subcommand_used(triangleMeshCmd)) {
        const int         resolution   = triangleMeshCmd.get<int>("--resolution");
        const int         padding      = triangleMeshCmd.get<int>("--padding-voxels");
        const double      scale        = triangleMeshCmd.get<double>("--scale");
        const auto        offset       = triangleMeshCmd.get<std::vector<double>>("--offset");
        const std::string classifyName = triangleMeshCmd.get<std::string>("--classify-mode");
        const auto        classifyMode = parseTriangleMeshClassifyMode(classifyName);
        const double      E            = triangleMeshCmd.get<double>("--E");
        const double      nu           = triangleMeshCmd.get<double>("--nu");
        const double      density      = triangleMeshCmd.get<double>("--density");
        const std::string inputMesh    = triangleMeshCmd.get<std::string>("--input-mesh");
        const std::string outputMesh   = triangleMeshCmd.get<std::string>("--output-mesh");

        int maxNumVoxels = 0;
        if (cubic_mesher::computeNumVoxels(resolution, maxNumVoxels) == false) {
            SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(),
                                "Resolution must be positive and small enough that N^3 fits in a signed int. Got N={}",
                                resolution);
            return 1;
        }

        if (!classifyMode.has_value()) {
            SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(),
                                "Unsupported classify mode '{}'. Expected one of: center, conservative",
                                classifyName);
            return 1;
        }

        if (scale <= 0.0) {
            SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "Triangle-mesh transform requires positive scale. Got scale={}",
                                scale);
            return 1;
        }

        cubic_mesher::TriangleMeshVoxelizerOptions options;
        options.inputMesh     = inputMesh;
        options.resolution    = resolution;
        options.paddingVoxels = padding;
        options.classifyMode  = *classifyMode;
        options.scale         = scale;
        options.offset        = {offset[0], offset[1], offset[2]};
        options.E             = E;
        options.nu            = nu;
        options.density       = density;

        std::unique_ptr<pgo::VolumetricMeshes::CubicMesh> cubicMesh = cubic_mesher::createTriangleMeshCubicMesh(options);
        if (cubicMesh == nullptr) {
            return 1;
        }

        if (cubic_mesher::saveAndValidateCubicMesh(*cubicMesh, outputMesh) == false) {
            return 1;
        }

        if (triangleMeshCmd.is_used("--output-surface")) {
            const std::string outputSurface = triangleMeshCmd.get<std::string>("--output-surface");
            if (cubic_mesher::writeSurfaceMesh(*cubicMesh, outputSurface) == false) {
                return 1;
            }
        }

        SPDLOG_LOGGER_INFO(
            pgo::Logging::lgr(),
            "Generated triangle-mesh cubic mesh: input={}, N={}, padding={}, classify={}, scale={}, offset=({}, {}, {}), vertices={}, elements={} -> {}",
            inputMesh, resolution, padding, cubic_mesher::toString(*classifyMode), scale, offset[0], offset[1],
            offset[2], cubicMesh->getNumVertices(), cubicMesh->getNumElements(), outputMesh);

        return 0;
    }

    return 0;
}
