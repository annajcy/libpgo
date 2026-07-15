add_executable(eigen_accelerate_nested_benchmark
  eigen_accelerate_nested_benchmark.cpp
  eigen_accelerate_nested_kernel.cpp
  eigen_accelerate_nested_kernel.h
)

target_link_libraries(eigen_accelerate_nested_benchmark
  PRIVATE
    benchmark::benchmark
    Eigen3::Eigen
    parallel
    TBB::tbb
)
