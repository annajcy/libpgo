if(TARGET openvdb_static OR TARGET OpenVDB::openvdb)
  return()
endif()

message(STATUS "Downloading OpenVDB...")
include(FetchContent)

# Build only the static OpenVDB core library. External dependencies are
# limited to TBB (provided by uv mkl-devel on Linux/Windows and Homebrew on
# macOS); compression, image, logging, and OpenEXR support are disabled so the
# build remains self-contained.
set(OPENVDB_BUILD_CORE ON CACHE BOOL "Build OpenVDB core library" FORCE)
set(OPENVDB_BUILD_BINARIES OFF CACHE BOOL "Disable OpenVDB binaries" FORCE)
set(OPENVDB_BUILD_PYTHON_MODULE OFF CACHE BOOL "Disable OpenVDB Python module" FORCE)
set(OPENVDB_BUILD_UNITTESTS OFF CACHE BOOL "Disable OpenVDB unit tests" FORCE)
set(OPENVDB_BUILD_DOCS OFF CACHE BOOL "Disable OpenVDB docs" FORCE)
set(OPENVDB_BUILD_HOUDINI_PLUGIN OFF CACHE BOOL "Disable Houdini plugin" FORCE)
set(OPENVDB_BUILD_MAYA_PLUGIN OFF CACHE BOOL "Disable Maya plugin" FORCE)
set(OPENVDB_BUILD_AX OFF CACHE BOOL "Disable OpenVDB AX" FORCE)
set(OPENVDB_BUILD_NANOVDB OFF CACHE BOOL "Disable NanoVDB" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static OpenVDB libraries" FORCE)
set(USE_TBB ON CACHE BOOL "Use TBB" FORCE)
set(USE_BLOSC OFF CACHE BOOL "Disable Blosc" FORCE)
set(USE_ZLIB OFF CACHE BOOL "Disable zlib" FORCE)
set(USE_LOG4CPLUS OFF CACHE BOOL "Disable log4cplus" FORCE)
set(USE_EXR OFF CACHE BOOL "Disable OpenEXR" FORCE)
set(USE_IMATH_HALF OFF CACHE BOOL "Disable Imath half" FORCE)
set(USE_PNG OFF CACHE BOOL "Disable PNG" FORCE)
set(OPENVDB_CXX_STRICT OFF CACHE BOOL "Disable strict C++ warnings" FORCE)
set(OPENVDB_ENABLE_ASSERTS OFF CACHE BOOL "Disable OpenVDB asserts" FORCE)
set(OPENVDB_FUTURE_DEPRECATION OFF CACHE BOOL "Disable future deprecation warnings" FORCE)
set(OPENVDB_ENABLE_UNINSTALL OFF CACHE BOOL
  "Disable OpenVDB uninstall target (Eigen already defines one)" FORCE)
set(DISABLE_DEPENDENCY_VERSION_CHECKS ON CACHE BOOL
  "Disable OpenVDB dependency version checks" FORCE)

FetchContent_Declare(
  openvdb
  URL https://github.com/AcademySoftwareFoundation/openvdb/archive/refs/tags/v12.0.0.tar.gz
  URL_HASH SHA256=23ceb5b18a851f45af118f718a9dd3001efaee364e3f623c37ffbdad03b8905f
  DOWNLOAD_EXTRACT_TIMESTAMP ON
  EXCLUDE_FROM_ALL
)

FetchContent_GetProperties(openvdb)
if(NOT openvdb_POPULATED)
  FetchContent_Populate(openvdb)
endif()

if(NOT openvdb_SOURCE_DIR)
  if(NOT FETCHCONTENT_BASE_DIR)
    set(FETCHCONTENT_BASE_DIR "${CMAKE_BINARY_DIR}/_deps")
  endif()
  set(openvdb_SOURCE_DIR "${FETCHCONTENT_BASE_DIR}/openvdb-src")
endif()

# oneTBB headers do not always define TBB_INTERFACE_VERSION before
# Threading.h's version check, so the old tbb::task::self() path can be
# selected even when oneTBB 2021+ is installed. Include version.h explicitly.
pgo_replace_in_file(
  "${openvdb_SOURCE_DIR}/openvdb/openvdb/thread/Threading.h"
  "#include <tbb/task_group.h>"
  "#include <tbb/task_group.h>\n#include <tbb/version.h>")

# NodeManager.h uses the template disambiguator on non-template static
# functions. Modern AppleClang rejects that form, so drop the disambiguator.
pgo_replace_in_file(
  "${openvdb_SOURCE_DIR}/openvdb/openvdb/tree/NodeManager.h"
  "OpT::template eval"
  "OpT::eval")

add_subdirectory("${openvdb_SOURCE_DIR}" "${openvdb_BINARY_DIR}" EXCLUDE_FROM_ALL)

if(NOT TARGET openvdb_static AND NOT TARGET OpenVDB::openvdb)
  message(FATAL_ERROR
    "OpenVDB FetchContent build completed without openvdb_static.")
endif()

message(STATUS "Done.")
