if(TARGET cuco)
  return()
endif()

pgo_dep_option(BUILD_TESTS BOOL OFF "Configure CMake to build tests")
pgo_dep_option(BUILD_BENCHMARKS BOOL OFF "Configure CMake to build (google) benchmarks")
pgo_dep_option(BUILD_EXAMPLES BOOL OFF "Configure CMake to build examples")
pgo_dep_option(BUILD_CUCO_TESTS BOOL OFF "Configure CMake to build cuco tests")

pgo_add_third_party(cuco
  TARGETS cuco
  STATUS "Loading cuco..."
  FETCHCONTENT_ARGS
    GIT_REPOSITORY https://github.com/NVIDIA/cuCollections.git
    GIT_TAG dev
    EXCLUDE_FROM_ALL
    DOWNLOAD_EXTRACT_TIMESTAMP ON
)
