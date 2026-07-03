if(TARGET autodiff::autodiff)
  return()
endif()

pgo_dep_option(AUTODIFF_BUILD_TESTS BOOL OFF "Enable the compilation of the test files.")
pgo_dep_option(AUTODIFF_BUILD_PYTHON BOOL OFF "Enable the compilation of the python bindings.")
pgo_dep_option(AUTODIFF_BUILD_EXAMPLES BOOL OFF "Enable the compilation of the example files.")
pgo_dep_option(AUTODIFF_BUILD_DOCS BOOL OFF "Enable the build of the documentation and website.")

pgo_add_third_party(autodiff_pkg
  TARGETS autodiff::autodiff
  STATUS "Loading autodiff..."
  FETCHCONTENT_ARGS
    URL https://github.com/autodiff/autodiff/archive/refs/tags/v1.1.2.tar.gz
    EXCLUDE_FROM_ALL
    DOWNLOAD_EXTRACT_TIMESTAMP ON
)
