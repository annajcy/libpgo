add_libpgo_tools(runSim runSim.cpp simulationApi)

target_link_libraries(runSim PRIVATE argparse::argparse)