add_executable(eigen_mkl_task_profile_probe
  eigen_mkl_task_profile_probe.cpp
  ../eigen_mkl_nested/eigen_mkl_nested_kernel.cpp
  ../eigen_mkl_nested/eigen_mkl_nested_kernel.h
)

target_link_libraries(eigen_mkl_task_profile_probe
  PRIVATE
    Eigen3::Eigen
    MKL::MKL
    parallel
    TBB::tbb
)
