if(TARGET benchmark::benchmark)
  return()
endif()

pgo_dep_option(BENCHMARK_ENABLE_TESTING BOOL OFF "Build Google Benchmark tests")
pgo_dep_option(BENCHMARK_ENABLE_GTEST_TESTS BOOL OFF "Build Google Benchmark GTest tests")
pgo_dep_option(BENCHMARK_ENABLE_INSTALL BOOL OFF "Generate Google Benchmark install targets")
pgo_dep_option(BENCHMARK_INSTALL_DOCS BOOL OFF "Install Google Benchmark documentation")
pgo_dep_option(BENCHMARK_ENABLE_DOXYGEN BOOL OFF "Build Google Benchmark documentation")
pgo_dep_option(BENCHMARK_DOWNLOAD_DEPENDENCIES BOOL OFF "Download Google Benchmark test dependencies")

pgo_add_third_party(benchmark
  TARGETS benchmark::benchmark
  STATUS "Loading Google Benchmark..."
  FETCHCONTENT_ARGS
    URL https://github.com/google/benchmark/archive/refs/tags/v1.9.5.zip
    EXCLUDE_FROM_ALL
    DOWNLOAD_EXTRACT_TIMESTAMP ON
)
