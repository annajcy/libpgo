add_executable(eigen_mkl_crossover_benchmark
  eigen_mkl_crossover_benchmark.cpp
  ../eigen_blas/eigen_gemm_kernel.cpp
  ../eigen_blas/eigen_gemm_kernel.h
  ../eigen_mkl_common/eigen_mkl_executor_cases.h
)

target_compile_definitions(eigen_mkl_crossover_benchmark
  PRIVATE PGO_EIGEN_BLAS_MKL
)

target_link_libraries(eigen_mkl_crossover_benchmark
  PRIVATE
    Eigen3::Eigen
    MKL::MKL
    parallel
    TBB::tbb
)
