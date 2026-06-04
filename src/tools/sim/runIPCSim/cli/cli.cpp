#include "cli/cli.h"

#include <argparse/argparse.hpp>

#include <iostream>
#include <stdexcept>
#include <string>

namespace pgo::RunIPCSim
{
namespace
{
void configureRunIPCSimArgumentParser(argparse::ArgumentParser &program)
{
  program.add_argument("config")
    .help("Config File")
    .required();
  program.add_argument("--log")
    .help("Write command-line output to a .log file next to the config file")
    .default_value(false)
    .implicit_value(true);
  program.add_argument("--contact-model")
    .help("Contact model: ipc or sampled-penalty")
    .default_value(std::string("ipc"));
}

RunIPCSimCliOptions readRunIPCSimCliOptions(const argparse::ArgumentParser &program)
{
  RunIPCSimCliOptions options;
  options.configPath = program.get<std::string>("config");
  options.runOptions.enableCliLog = program.get<bool>("--log");
  const std::string contactModel = program.get<std::string>("--contact-model");
  if (contactModel == "ipc") {
    options.runOptions.contactBackendKind = ContactBackendKind::Ipc;
  }
  else if (contactModel == "sampled-penalty") {
    options.runOptions.contactBackendKind = ContactBackendKind::SampledPenalty;
  }
  else {
    throw std::invalid_argument("runIPCSim --contact-model expects `ipc` or `sampled-penalty`.");
  }
  return options;
}
}  // namespace

RunIPCSimCliOptions parseRunIPCSimCli(int argc, char *argv[])
{
  argparse::ArgumentParser program("Run IPC Simulation");
  configureRunIPCSimArgumentParser(program);
  program.parse_args(argc, argv);
  return readRunIPCSimCliOptions(program);
}

int runCli(int argc, char *argv[])
{
  argparse::ArgumentParser program("Run IPC Simulation");
  configureRunIPCSimArgumentParser(program);
  try {
    program.parse_args(argc, argv);
  }
  catch (const std::exception &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  const RunIPCSimCliOptions options = readRunIPCSimCliOptions(program);
  return runFromConfig(options.configPath, options.runOptions);
}
}  // namespace pgo::RunIPCSim
