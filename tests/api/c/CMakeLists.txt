if(NOT TARGET pgo_c)
  message(STATUS "api_pgo_c_test is not included. Missing target: pgo_c")
  return()
endif()

set(AVAILABLE_LIBS)

foreach(lib ${PGO_GLOBAL_LIBRARY_TARGETS})
  if(TARGET ${lib})
    list(APPEND AVAILABLE_LIBS ${lib})
  endif()
endforeach()

add_libpgo_tools(api_pgo_c_test pgo_c_test.cpp "${AVAILABLE_LIBS}")
target_link_libraries(api_pgo_c_test PRIVATE GTest::gtest_main pgo_c)
target_include_directories(api_pgo_c_test PRIVATE ${PROJECT_SOURCE_DIR}/src/api/c)
set_property(TARGET api_pgo_c_test PROPERTY FOLDER "tests")

add_test(NAME api.pgo_c.basic COMMAND api_pgo_c_test)