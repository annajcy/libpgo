add_executable(eigen_mkl_control_matrix_benchmark
  eigen_mkl_control_matrix_benchmark.cpp
  ../eigen_mkl_nested/eigen_mkl_nested_kernel.cpp
  ../eigen_mkl_nested/eigen_mkl_nested_kernel.h
)

target_link_libraries(eigen_mkl_control_matrix_benchmark
  PRIVATE
    benchmark::benchmark
    Eigen3::Eigen
    MKL::MKL
    parallel
    TBB::tbb
)
