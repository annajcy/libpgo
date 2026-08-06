cmake_minimum_required(VERSION 3.28.0 FATAL_ERROR)

# Some pinned FetchContent dependencies (GMSH 4.13) still declare
# cmake_minimum_required(VERSION 3.3). CMake 4.x removes compatibility below
# 3.5; this variable lets those old projects configure with the bundled CMake.
set(CMAKE_POLICY_VERSION_MINIMUM "3.5" CACHE STRING
  "Minimum CMake policy version accepted from FetchContent dependencies" FORCE)

include(CMakeModules/projectVersion.cmake)
pgo_read_project_version(
  "${CMAKE_CURRENT_SOURCE_DIR}/pyproject.toml"
  PGO_PROJECT_VERSION)

project(libpgo VERSION "${PGO_PROJECT_VERSION}" LANGUAGES CXX C)

if(NOT DEFINED PYPGO_VERSION_INFO)
  set(PYPGO_VERSION_INFO "${PROJECT_VERSION}")
endif()

include(CMakeModules/PgoOptions.cmake)
include(CMakeModules/PgoPlatform.cmake)
include(CMakeModules/releaseBuildPolicy.cmake)

set(BUILD_TESTING OFF CACHE BOOL "Disable third-party tests during dependency configuration" FORCE)
set(PGO_CLANGD_COMPILE_COMMANDS "${CMAKE_SOURCE_DIR}/compile_commands.json")

if(APPLE AND PGO_ENABLE_PYTHON AND PGO_RUNTIME_LAYOUT STREQUAL "WHEEL"
   AND NOT CMAKE_OSX_DEPLOYMENT_TARGET)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "26.0" CACHE STRING
    "Minimum macOS version for Python extension wheel builds")
endif()

include(CMakeModules/pythonDependencies.cmake)
pgo_configure_python_dependencies()

include(CMakeModules/runtimeDependencies.cmake)
pgo_configure_runtime_dependencies()

add_custom_target(copy_compile_commands_json
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_SOURCE_DIR}/build"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${CMAKE_BINARY_DIR}/compile_commands.json"
          "${PGO_CLANGD_COMPILE_COMMANDS}"
  BYPRODUCTS "${PGO_CLANGD_COMPILE_COMMANDS}"
  COMMENT "Copying compile_commands.json to ${PGO_CLANGD_COMPILE_COMMANDS}"
  VERBATIM
)

if (PGO_ENABLE_CUDA)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(EXISTS "/usr/bin/g++")
      message(STATUS "Set CUDA host compiler: /usr/bin/g++")
      set(CMAKE_CUDA_HOST_COMPILER /usr/bin/g++)
    endif()
  endif()

  set(CMAKE_CUDA_ARCHITECTURES native)
  enable_language(CUDA)
endif()

# message(STATUS "CMake module path: ${CMAKE_MODULE_PATH}")
set(PGO_CMAKE_MODULES ${CMAKE_SOURCE_DIR}/CMakeModules)
list(APPEND CMAKE_MODULE_PATH "${PGO_CMAKE_MODULES}")
message(STATUS "CMake Module Path: ${CMAKE_MODULE_PATH}")

include(CompilationUtils)
include(PgoDependencyHelpers)

include(CMakeModules/PgoCompilerFlags.cmake)

if(APPLE)
  find_program(
    _pgo_homebrew_executable
    NAMES brew
    HINTS /opt/homebrew/bin /usr/local/bin
    NO_CACHE
  )
  if(NOT _pgo_homebrew_executable)
    message(FATAL_ERROR
      "Homebrew is required to locate OpenMP on macOS. "
      "Install Homebrew, then run: brew install libomp")
  endif()

  execute_process(
    COMMAND "${_pgo_homebrew_executable}" --prefix libomp
    OUTPUT_VARIABLE _pgo_homebrew_libomp_prefix
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _pgo_homebrew_libomp_result
    ERROR_QUIET
  )
  if(NOT _pgo_homebrew_libomp_result EQUAL 0
     OR NOT EXISTS "${_pgo_homebrew_libomp_prefix}/include/omp.h"
     OR NOT EXISTS "${_pgo_homebrew_libomp_prefix}/lib/libomp.dylib")
    message(FATAL_ERROR
      "Homebrew libomp is required on macOS. Run: brew install libomp")
  endif()

  list(PREPEND CMAKE_PREFIX_PATH "${_pgo_homebrew_libomp_prefix}")
  set(OpenMP_CXX_INCLUDE_DIR
    "${_pgo_homebrew_libomp_prefix}/include"
    CACHE PATH "AppleClang OpenMP include directory" FORCE)
  set(OpenMP_libomp_LIBRARY
    "${_pgo_homebrew_libomp_prefix}/lib/libomp.dylib"
    CACHE FILEPATH "AppleClang OpenMP runtime" FORCE)
  message(STATUS "Using Homebrew libomp: ${_pgo_homebrew_libomp_prefix}")

  unset(_pgo_homebrew_executable)
  unset(_pgo_homebrew_libomp_prefix)
  unset(_pgo_homebrew_libomp_result)
endif()

find_package(OpenMP REQUIRED COMPONENTS CXX)
message(STATUS "OpenMP CXX found: ${OpenMP_CXX_FLAGS}")

if(TARGET OpenMP::OpenMP_CXX)
  target_link_libraries(compilation_flag INTERFACE OpenMP::OpenMP_CXX)
  target_compile_definitions(compilation_flag INTERFACE USE_OPENMP)
endif()

include(CMakeModules/PgoThirdParty.cmake)
include(CMakeModules/PgoTargets.cmake)

add_subdirectory(src)

if(PGO_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

if(PGO_BUILD_TESTING)
  set(BUILD_TESTING ON CACHE BOOL "Enable libpgo tests after third-party dependency configuration" FORCE)
  include(CTest)
  message(STATUS "Testing enabled.")
  add_subdirectory(tests)
endif()

if(PGO_BUILD_SUBPROJECTS)
  add_subdirectory(projects)
endif()
