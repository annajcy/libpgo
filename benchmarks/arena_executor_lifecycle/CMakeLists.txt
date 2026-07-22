add_executable(arena_executor_lifecycle_benchmark
  arena_executor_lifecycle_benchmark.cpp
)

target_link_libraries(arena_executor_lifecycle_benchmark
  PRIVATE
    parallel
    TBB::tbb
)
