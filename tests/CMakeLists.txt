include(FetchContent)
include(GoogleTest)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_test(
  NAME tbb_scheduling_inventory
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/check_tbb_scheduling_inventory.py
          --repo-root ${CMAKE_SOURCE_DIR}
)
add_test(
  NAME tbb_scheduling_inventory_self_test
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/check_tbb_scheduling_inventory.py
          --self-test
)

enable_testing()

set(INSTALL_GTEST OFF CACHE BOOL "Disable GoogleTest installation" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "Disable GoogleMock build" FORCE)
set(gtest_force_shared_crt ON CACHE BOOL "Use shared CRT for GoogleTest on Windows" FORCE)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
  DOWNLOAD_EXTRACT_TIMESTAMP ON
)

pgo_fetch_make_available(googletest)

function(pgo_gtest_discover_tests target)
  if(WIN32 AND TARGET pgo_c)
    set_property(TARGET ${target} PROPERTY TEST_LAUNCHER
      "${CMAKE_COMMAND};-E;env;--modify;PATH=path_list_prepend:$<TARGET_FILE_DIR:pgo_c>")
  endif()

  gtest_discover_tests(${target}
    DISCOVERY_MODE PRE_TEST
    DISCOVERY_TIMEOUT 30
    ${ARGN})
endfunction()

function(pgo_add_gtest target)
  cmake_parse_arguments(PGO_GTEST "" "" "SOURCES;DEPS;INCLUDE_DIRS;DEFINITIONS" ${ARGN})

  add_executable(${target} ${PGO_GTEST_SOURCES})
  if(PGO_GTEST_INCLUDE_DIRS)
    target_include_directories(${target} PRIVATE ${PGO_GTEST_INCLUDE_DIRS})
  endif()
  target_link_libraries(${target} PRIVATE GTest::gtest_main ${PGO_GTEST_DEPS})
  if(PGO_GTEST_DEFINITIONS)
    target_compile_definitions(${target} PRIVATE ${PGO_GTEST_DEFINITIONS})
  endif()
  set_property(TARGET ${target} PROPERTY FOLDER "tests/gtest")
  pgo_gtest_discover_tests(${target})
endfunction()

pgo_add_gtest(pgo_c_gtest
  SOURCES pgo_c_gtest.cpp
  DEPS pgo_c
  INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/src/c
  DEFINITIONS LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/examples/assets/veg/tet/torus.veg"
)
if(TARGET MKL::MKL)
  target_compile_definitions(pgo_c_gtest PRIVATE PGO_HAS_MKL)
endif()

foreach(concurrency IN ITEMS 1 4)
  set(target pgo_c_parallel_${concurrency}_gtest)
  pgo_add_gtest(${target}
    SOURCES pgo_c_parallel_gtest.cpp
    DEPS pgo_c parallel
    INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/src/c ${CMAKE_SOURCE_DIR}/src/core
    DEFINITIONS PGO_TEST_MAX_CONCURRENCY=${concurrency}
  )
endforeach()

add_subdirectory(src)
