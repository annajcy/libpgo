get_target_property(PGO_EIGEN_BENCHMARK_INCLUDE_DIRS
  Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
if(NOT PGO_EIGEN_BENCHMARK_INCLUDE_DIRS)
  message(FATAL_ERROR "Eigen3::Eigen does not expose include directories for the no-BLAS benchmark.")
endif()

set(PGO_EIGEN_BLAS_COMMON_SOURCES
  eigen_gemm_benchmark.cpp
  eigen_gemm_kernel.cpp
  eigen_gemm_kernel.h
  backend_scope.h
)

function(pgo_add_eigen_blas_benchmark target backend_source backend_definition)
  add_executable(${target}
    ${PGO_EIGEN_BLAS_COMMON_SOURCES}
    ${backend_source}
  )
  target_link_libraries(${target} PRIVATE benchmark::benchmark)
  target_compile_definitions(${target} PRIVATE ${backend_definition})
endfunction()

pgo_add_eigen_blas_benchmark(
  eigen_internal_gemm_benchmark
  backend_scope_internal.cpp
  PGO_EIGEN_BLAS_INTERNAL)
target_include_directories(eigen_internal_gemm_benchmark
  SYSTEM PRIVATE ${PGO_EIGEN_BENCHMARK_INCLUDE_DIRS})
target_compile_definitions(eigen_internal_gemm_benchmark
  PRIVATE EIGEN_DONT_PARALLELIZE)

if(APPLE)
  pgo_add_eigen_blas_benchmark(
    eigen_accelerate_gemm_benchmark
    backend_scope_accelerate.cpp
    PGO_EIGEN_BLAS_ACCELERATE)
  target_link_libraries(eigen_accelerate_gemm_benchmark
    PRIVATE
      Eigen3::Eigen
      parallel)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
   TARGET MKL::MKL AND
   PGO_MKL_THREADING STREQUAL "tbb_thread")
  pgo_add_eigen_blas_benchmark(
    eigen_mkl_gemm_benchmark
    backend_scope_mkl.cpp
    PGO_EIGEN_BLAS_MKL)
  target_sources(eigen_mkl_gemm_benchmark PRIVATE
    ../eigen_mkl_common/eigen_mkl_executor_cases.h)
  target_link_libraries(eigen_mkl_gemm_benchmark
    PRIVATE
      Eigen3::Eigen
      MKL::MKL
      parallel
      TBB::tbb)
endif()
