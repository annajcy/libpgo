set(AVAILABLE_LIBS)

foreach(lib ${PGO_GLOBAL_LIBRARY_TARGETS})
  if(TARGET ${lib})
    list(APPEND AVAILABLE_LIBS ${lib})
  endif()
endforeach()

add_libpgo_tools(cubicMesher_tool_test cubicMesher_test.cpp "${AVAILABLE_LIBS}")
target_link_libraries(cubicMesher_tool_test PRIVATE GTest::gtest_main)
add_dependencies(cubicMesher_tool_test cubicMesher)
target_compile_definitions(cubicMesher_tool_test PRIVATE
  PGO_TEST_CUBIC_MESHER_BIN="$<TARGET_FILE:cubicMesher>")
set_property(TARGET cubicMesher_tool_test PROPERTY FOLDER "tests")

add_test(NAME cubicMesher.tool.uniform COMMAND cubicMesher_tool_test)
set_tests_properties(cubicMesher.tool.uniform
  PROPERTIES ENVIRONMENT "CUBIC_MESHER_BIN=$<TARGET_FILE:cubicMesher>")

if(TARGET runSim)
  add_libpgo_tools(runSim_tool_test runSim_test.cpp "")
  target_link_libraries(runSim_tool_test PRIVATE GTest::gtest_main)
  add_dependencies(runSim_tool_test runSim)
  target_compile_definitions(runSim_tool_test PRIVATE
    PGO_TEST_RUN_SIM_BIN="$<TARGET_FILE:runSim>")
  set_property(TARGET runSim_tool_test PROPERTY FOLDER "tests")

  add_test(NAME runSim.tool.path_resolution COMMAND runSim_tool_test)
  set_tests_properties(runSim.tool.path_resolution
    PROPERTIES ENVIRONMENT "RUN_SIM_BIN=$<TARGET_FILE:runSim>")
else()
  message(STATUS "runSim_tool_test is not included. Missing target: runSim")
endif()
