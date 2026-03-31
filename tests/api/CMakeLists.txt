add_subdirectory(c)

if(TARGET simulationApi)
  add_libpgo_tools(api_runSim_config_parse_test runSimConfig_parse_test.cpp simulationApi)
  target_link_libraries(api_runSim_config_parse_test PRIVATE GTest::gtest_main)
  set_property(TARGET api_runSim_config_parse_test PROPERTY FOLDER "tests")

  add_test(NAME api.runSim_config.parse COMMAND api_runSim_config_parse_test)
else()
  message(STATUS "api_runSim_config_parse_test is not included. Missing target: simulationApi")
endif()
