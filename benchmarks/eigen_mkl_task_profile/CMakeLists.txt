add_executable(eigen_mkl_task_profile_probe
  eigen_mkl_task_profile_probe.cpp
  ../eigen_mkl_common/eigen_mkl_executor_cases.h
  ../eigen_mkl_common/eigen_mkl_gemm_workload.cpp
  ../eigen_mkl_common/eigen_mkl_gemm_workload.h
)

target_link_libraries(eigen_mkl_task_profile_probe
  PRIVATE
    Eigen3::Eigen
    MKL::MKL
    parallel
    TBB::tbb
)
