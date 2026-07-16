add_executable(eigen_mkl_budget_sweep_probe
  eigen_mkl_budget_sweep_probe.cpp
  ../eigen_mkl_common/eigen_mkl_gemm_workload.cpp
  ../eigen_mkl_common/eigen_mkl_gemm_workload.h
)

target_link_libraries(eigen_mkl_budget_sweep_probe
  PRIVATE
    Eigen3::Eigen
    MKL::MKL
    parallel
    TBB::tbb
)
