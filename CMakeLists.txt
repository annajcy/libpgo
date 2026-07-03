cmake_minimum_required(VERSION 3.28.0 FATAL_ERROR)
project(libpgo LANGUAGES CXX C)

include(CMakeModules/PgoOptions.cmake)
include(CMakeModules/PgoPlatform.cmake)

set(BUILD_TESTING OFF CACHE BOOL "Disable third-party tests during dependency configuration" FORCE)
set(PGO_CLANGD_COMPILE_COMMANDS "${CMAKE_SOURCE_DIR}/compile_commands.json")

add_custom_target(copy_compile_commands_json
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
    else()
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
include(CMakeModules/PgoThirdParty.cmake)
include(CMakeModules/PgoTargets.cmake)

add_subdirectory(src)

if(PGO_BUILD_TESTING)
  include(CTest)
  message(STATUS "Testing enabled.")
  add_subdirectory(tests)
endif()

if(PGO_BUILD_SUBPROJECTS)
  add_subdirectory(projects)
endif()

# generate config file to be used in common build tree
configure_package_config_file(
  CMakeModules/pgoConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/pgoConfig.cmake
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/pgo/"
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/pgoConfig.cmake
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/pgo/"
)
