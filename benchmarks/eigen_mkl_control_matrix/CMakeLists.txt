add_executable(eigen_mkl_control_matrix_benchmark
  eigen_mkl_control_matrix_benchmark.cpp
  ../eigen_mkl_common/eigen_mkl_executor_cases.h
  ../eigen_mkl_common/eigen_mkl_gemm_workload.cpp
  ../eigen_mkl_common/eigen_mkl_gemm_workload.h
)

pgo_add_mkl_tbb_probe(eigen_mkl_control_matrix_probe
  eigen_mkl_control_matrix_probe.cpp
  ../eigen_mkl_common/eigen_mkl_executor_cases.h
  ../eigen_mkl_common/eigen_mkl_gemm_workload.cpp
  ../eigen_mkl_common/eigen_mkl_gemm_workload.h
)

target_link_libraries(eigen_mkl_control_matrix_benchmark
  PRIVATE
    Eigen3::Eigen
    MKL::MKL
    parallel
    TBB::tbb
)
