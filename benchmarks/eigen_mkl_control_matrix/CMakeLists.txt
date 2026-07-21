add_executable(eigen_mkl_control_matrix_benchmark
  eigen_mkl_control_matrix_benchmark.cpp
  ../eigen_mkl_common/eigen_mkl_executor_cases.h
  ../eigen_mkl_common/eigen_mkl_gemm_workload.cpp
  ../eigen_mkl_common/eigen_mkl_gemm_workload.h
)

get_target_property(eigen_include_directories Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
if(NOT eigen_include_directories)
  message(FATAL_ERROR "Eigen3::Eigen must expose INTERFACE_INCLUDE_DIRECTORIES.")
endif()

add_executable(eigen_mkl_control_matrix_no_blas_benchmark
  eigen_mkl_control_matrix_benchmark.cpp
  ../eigen_mkl_common/eigen_mkl_executor_cases.h
  no_blas_workload.cpp
  no_blas_workload.h
)

target_include_directories(eigen_mkl_control_matrix_no_blas_benchmark
  PRIVATE
    ${eigen_include_directories}
)

target_compile_definitions(eigen_mkl_control_matrix_no_blas_benchmark
  PRIVATE
    PGO_EIGEN_MKL_CONTROL_NO_BLAS
    EIGEN_DONT_PARALLELIZE
    EIGEN_MAX_ALIGN_BYTES=32
)

target_link_libraries(eigen_mkl_control_matrix_no_blas_benchmark
  PRIVATE
    MKL::MKL
    parallel
    TBB::tbb
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
