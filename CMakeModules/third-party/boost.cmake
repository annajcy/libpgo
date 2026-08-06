if(TARGET Boost::boost)
  return()
endif()

message(STATUS "Loading Boost...")
include(FetchContent)

FetchContent_Declare(
  boost
  URL https://github.com/boostorg/boost/releases/download/boost-1.85.0/boost-1.85.0-cmake.tar.xz
  URL_HASH SHA256=0a9cc56ceae46986f5f4d43fe0311d90cf6d2fa9028258a95cab49ffdacf92ad
  EXCLUDE_FROM_ALL
  DOWNLOAD_EXTRACT_TIMESTAMP ON
  OVERRIDE_FIND_PACKAGE
)

FetchContent_MakeAvailable(boost)

# The boost-cmake superproject places each library's headers under
# libs/<name>/include. OpenVDB includes boost/interprocess (and its transitive
# headers) directly while only linking Boost::iostreams, whose interface
# include set does not cover them. Expose every Boost module include dir on
# that target so the whole fetched Boost tree is usable by dependencies.
if(TARGET Boost::iostreams)
  get_target_property(_pgo_boost_iostreams_target Boost::iostreams ALIASED_TARGET)
  if(NOT _pgo_boost_iostreams_target)
    set(_pgo_boost_iostreams_target Boost::iostreams)
  endif()
  file(GLOB _pgo_boost_module_include_dirs
    LIST_DIRECTORIES true
    "${boost_SOURCE_DIR}/libs/*/include")
  target_include_directories(${_pgo_boost_iostreams_target} INTERFACE
    ${_pgo_boost_module_include_dirs})
endif()

message(STATUS "Done.")
