if(NOT TARGET benchmark::benchmark)
  message(STATUS "parallelism benchmarks are skipped. Missing: benchmark::benchmark")
  return()
endif()

if(NOT TARGET TBB::tbb)
  message(STATUS "parallelism benchmarks are skipped. Missing: TBB::tbb")
  return()
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  if(NOT TARGET MKL::MKL)
    message(STATUS "parallelism_mkl_nested_benchmark is skipped. Missing: MKL::MKL")
  elseif(NOT PGO_MKL_THREADING STREQUAL "tbb_thread")
    message(STATUS "parallelism_mkl_nested_benchmark is skipped. PGO_MKL_THREADING is not tbb_thread.")
  else()
    add_executable(parallelism_mkl_nested_benchmark
      parallelism_mkl_nested_benchmark.cpp
    )

    target_link_libraries(parallelism_mkl_nested_benchmark
      PRIVATE
        benchmark::benchmark
        parallelism
        TBB::tbb
        MKL::MKL
    )

    target_compile_definitions(parallelism_mkl_nested_benchmark
      PRIVATE
        PGO_PARALLELISM_HAS_TBB
        PGO_PARALLELISM_HAS_MKL
        PGO_PARALLELISM_MKL_TBB_THREADING
    )
  endif()
endif()

if(APPLE)
  add_executable(parallelism_accelerate_nested_benchmark
    parallelism_accelerate_nested_benchmark.cpp
  )

  target_link_libraries(parallelism_accelerate_nested_benchmark
    PRIVATE
      benchmark::benchmark
      parallelism
      TBB::tbb
      "-framework Accelerate"
  )

  target_compile_definitions(parallelism_accelerate_nested_benchmark
    PRIVATE
      PGO_PARALLELISM_HAS_TBB
      PGO_PARALLELISM_HAS_ACCELERATE_THREADING
  )
endif()
