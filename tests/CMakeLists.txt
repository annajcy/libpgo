include(FetchContent)
include(GoogleTest)

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

add_subdirectory(src)
