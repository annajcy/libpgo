#include "argparse/argparse.hpp"
#include "configFileJSON.h"
#include "fileService.h"
#include "initPredicates.h"
#include "pgoLogging.h"
#include "runSimCore.h"

#include <exception>
#include <iostream>

int main(int argc, char* argv[]) {
    argparse::ArgumentParser program("Run Simulation");
    program.add_argument("config").help("Config File").default_value(std::string("examples/box/box.json"));
    program.add_argument("--deterministic")
        .help("Force deterministic mode by running the simulation single-threaded")
        .default_value(false)
        .implicit_value(true);

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    pgo::Logging::init();
    pgo::Mesh::initPredicates();

    const pgo::FileService::ConfigPathResolver configPathResolver(program.get<std::string>("config"));

    pgo::ConfigFileJSON jconfig;
    if (jconfig.open(configPathResolver.filePath().c_str()) != true) {
        return 0;
    }

    pgo::api::RunSimConfig config;
    try {
        config = pgo::api::parseRunSimConfig(jconfig, configPathResolver.filePath());
    } catch (const std::exception& err) {
        SPDLOG_LOGGER_ERROR(pgo::Logging::lgr(), "{}", err.what());
        return 1;
    }

    if (program.get<bool>("--deterministic")) {
        config.deterministicMode = true;
    }

    // config.restartEnabled = true;
    // config.restartPause = true;

    return pgo::api::runSimFromConfig(config);
}
