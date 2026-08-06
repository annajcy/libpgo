if(TARGET Ceres::Ceres)
  return()
endif()

pgo_dep_option(USE_CUDA BOOL OFF "use cuda")
pgo_dep_option(MINIGLOG BOOL ON "use mini glog")
pgo_dep_option(GFLAGS BOOL OFF "use gflags")
pgo_dep_option(BUILD_TESTING BOOL OFF "Enable tests")
pgo_dep_option(BUILD_DOCUMENTATION BOOL OFF "Build User's Guide (html)")
pgo_dep_option(BUILD_EXAMPLES BOOL OFF "Build examples")
pgo_dep_option(BUILD_BENCHMARKS BOOL OFF "Build Ceres benchmarking suite")
pgo_dep_option(BUILD_SHARED_LIBS BOOL OFF "Build Ceres as a shared library.")
pgo_dep_option(PROVIDE_UNINSTALL_TARGET BOOL OFF "Add a custom target to ease removal of installed targets")
pgo_dep_option(LAPACK BOOL OFF "Use LAPACK")
pgo_dep_option(SUITESPARSE BOOL OFF "Use SuiteSparse")
pgo_dep_option(ACCELERATESPARSE BOOL OFF "Use Apple Accelerate sparse solvers")
pgo_dep_option(EIGENSPARSE BOOL OFF "Use Eigen sparse solvers")

function(_pgo_patch_ceres_miniglog)
  # Windows.h defines ERROR as a macro, which breaks miniglog's
  # "const int ERROR = -2;" severity constant. Undefine it before the header
  # declares its own constants.
  pgo_replace_in_file(
    "${ceres_SOURCE_DIR}/internal/ceres/miniglog/glog/logging.h"
    "const int ERROR   = -2;"
    "#ifdef _WIN32\n#undef ERROR\n#endif\nconst int ERROR   = -2;")
endfunction()

pgo_add_third_party(ceres
  TARGETS Ceres::Ceres
  STATUS "Loading Ceres..."
  POST_FETCH _pgo_patch_ceres_miniglog
  FETCHCONTENT_ARGS
    URL http://ceres-solver.org/ceres-solver-2.2.0.tar.gz
    EXCLUDE_FROM_ALL
    DOWNLOAD_EXTRACT_TIMESTAMP ON
)
