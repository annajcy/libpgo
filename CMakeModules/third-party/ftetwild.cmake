set(_PGO_SAVED_CMAKE_CXX_STANDARD "${CMAKE_CXX_STANDARD}")
set(_PGO_SAVED_CMAKE_CXX_STANDARD_REQUIRED "${CMAKE_CXX_STANDARD_REQUIRED}")
set(_PGO_SAVED_CMAKE_CXX_EXTENSIONS "${CMAKE_CXX_EXTENSIONS}")

pgo_dep_option(FLOAT_TETWILD_ENABLE_TBB BOOL ON "Enable TBB in fTetWild")
pgo_dep_option(FLOAT_TETWILD_USE_FLOAT BOOL OFF "Use float precision in fTetWild")
pgo_dep_option(FLOAT_TETWILD_WITH_SANITIZERS BOOL OFF "Enable fTetWild sanitizers")
pgo_dep_option(FLOAT_TETWILD_WITH_EXACT_ENVELOPE BOOL OFF "Enable fTetWild exact envelope")

function(_libpgo_patch_ftetwild_geogram target_file)
  pgo_replace_in_file(
    "${target_file}"
    [=[add_custom_target(uninstall
COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)]=]
    [=[if(NOT TARGET uninstall)
add_custom_target(uninstall
COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()]=])
endfunction()

function(_libpgo_patch_ftetwild_geogram_openmp target_file)
  pgo_replace_in_file(
    "${target_file}"
    [=[if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")]=]
    [=[if(${CMAKE_SYSTEM_NAME} MATCHES "Linux" AND PGO_ENABLE_OPENMP)]=]
  )
endfunction()

function(_libpgo_patch_geogram_linux_openmp target_file)
  pgo_replace_in_file(
    "${target_file}"
    [=[if (GCC_VERSION VERSION_GREATER 4.0)
    add_flags(CMAKE_CXX_FLAGS -fopenmp)
    add_flags(CMAKE_C_FLAGS -fopenmp)
endif()]=]
    [=[if (GCC_VERSION VERSION_GREATER 4.0 AND PGO_ENABLE_OPENMP)
    add_flags(CMAKE_CXX_FLAGS -fopenmp)
    add_flags(CMAKE_C_FLAGS -fopenmp)
endif()]=]
  )
endfunction()

function(_libpgo_prepare_ftetwild_geogram)
  if(TARGET geogram)
    if(NOT TARGET geogram::geogram)
      add_library(geogram::geogram ALIAS geogram)
    endif()
    return()
  endif()

  if(MSVC)
    set(GEO_PLATFORM "Win-vs-generic")
  elseif(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(GEO_PLATFORM "Darwin-clang")
  else()
    set(GEO_PLATFORM "Linux64-gcc")
  endif()

  pgo_dep_option(VORPALINE_PLATFORM STRING "${GEO_PLATFORM}" "Geogram platform")
  pgo_dep_option(GEOGRAM_BUILD_SHARED BOOL OFF "Build geogram shared library")
  pgo_dep_option(GEOGRAM_BUILD_STATIC BOOL ON "Build geogram static library")
  pgo_dep_option(GEOGRAM_SUB_BUILD BOOL ON "Building as subproject")
  pgo_dep_option(GEOGRAM_LIB_ONLY BOOL ON "Build geogram lib only")
  pgo_dep_option(GEOGRAM_WITH_GRAPHICS BOOL OFF "Disable graphics")
  pgo_dep_option(GEOGRAM_WITH_LUA BOOL OFF "Disable LUA")
  pgo_dep_option(GEOGRAM_WITH_EXPLORAGRAM BOOL OFF "Disable exploragram")
  pgo_dep_option(GEOGRAM_WITH_LEGACY_NUMERICS BOOL OFF "Disable legacy numerics")
  pgo_dep_option(GEOGRAM_WITH_TRIANGLE BOOL OFF "Disable triangle")

  pgo_add_third_party(geogram
    TARGETS geogram
    FETCH_MODE POPULATE
    POPULATE_REASON "fTetWild patches geogram before add_subdirectory"
    POST_FETCH _libpgo_setup_ftetwild_geogram
    FETCHCONTENT_ARGS
      GIT_REPOSITORY https://github.com/BrunoLevy/geogram
      GIT_TAG v1.9.6
  )
endfunction()

function(_libpgo_setup_ftetwild_geogram)
  _libpgo_patch_ftetwild_geogram("${geogram_SOURCE_DIR}/CMakeLists.txt")
  _libpgo_patch_geogram_linux_openmp("${geogram_SOURCE_DIR}/cmake/platforms/Linux-gcc.cmake")
  pgo_add_populated_subdirectory(geogram)

  if(TARGET geogram AND NOT TARGET geogram::geogram)
    add_library(geogram::geogram ALIAS geogram)
  endif()
endfunction()

# fTetWild fetches Geogram itself; preloading a patched target avoids duplicate
# global targets such as Eigen's and Geogram's "uninstall".
_libpgo_prepare_ftetwild_geogram()

if(TARGET nlohmann_json::nlohmann_json AND NOT TARGET json)
  add_library(json INTERFACE)
  target_link_libraries(json INTERFACE nlohmann_json::nlohmann_json)
endif()

pgo_reuse_existing_fetchcontent_source(json)
pgo_reuse_existing_fetchcontent_source(predicates)

if(WIN32 AND GMP_INCLUDE_DIR AND NOT GMP_INCLUDE_DIRS)
  set(GMP_INCLUDE_DIRS "${GMP_INCLUDE_DIR}" CACHE PATH "Include path to GMP" FORCE)
endif()

function(_pgo_setup_ftetwild)
  _libpgo_patch_ftetwild_geogram_openmp("${ftetwild_SOURCE_DIR}/cmake/geogram.cmake")
  pgo_add_populated_subdirectory(ftetwild)
endfunction()

pgo_add_third_party(ftetwild
  TARGETS FloatTetwild
  STATUS "Loading fTetWild..."
  FETCH_MODE POPULATE
  POPULATE_REASON "fTetWild source tree is patched before add_subdirectory"
  POST_FETCH _pgo_setup_ftetwild
  FETCHCONTENT_ARGS
    GIT_REPOSITORY https://github.com/wildmeshing/fTetWild.git
    GIT_TAG d7d99bb4387a07895b9adce058dc7305f6b6e5ab
)

set(CMAKE_CXX_STANDARD "${_PGO_SAVED_CMAKE_CXX_STANDARD}")
set(CMAKE_CXX_STANDARD_REQUIRED "${_PGO_SAVED_CMAKE_CXX_STANDARD_REQUIRED}")
set(CMAKE_CXX_EXTENSIONS "${_PGO_SAVED_CMAKE_CXX_EXTENSIONS}")

if(NOT TARGET FloatTetwild)
  message(FATAL_ERROR "PGO_TET_MESHER_USE_TET_WILD=ON requires fTetWild target FloatTetwild, but it was not created.")
endif()

set(_PGO_FTETWILD_COMPAT_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/pgo_ftetwild_compat/include")
file(MAKE_DIRECTORY "${_PGO_FTETWILD_COMPAT_INCLUDE_DIR}/igl/predicates")
file(WRITE "${_PGO_FTETWILD_COMPAT_INCLUDE_DIR}/igl/predicates/predicates.h"
  "#pragma once\n"
  "#include <igl/Orientation.h>\n"
  "namespace igl { namespace predicates { using Orientation = ::igl::Orientation; } }\n"
  "#include <igl/predicates/exactinit.h>\n"
  "#include <igl/predicates/orient2d.h>\n"
  "#include <igl/predicates/orient3d.h>\n")
target_include_directories(FloatTetwild PRIVATE "${_PGO_FTETWILD_COMPAT_INCLUDE_DIR}")
