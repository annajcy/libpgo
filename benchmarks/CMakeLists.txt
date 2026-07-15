if(NOT TARGET benchmark::benchmark)
  message(STATUS "parallel benchmarks are skipped. Missing: benchmark::benchmark")
  return()
endif()

if(APPLE OR
   (CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
    TARGET MKL::MKL AND
    PGO_MKL_THREADING STREQUAL "tbb_thread"))
  add_subdirectory(eigen_blas)
endif()

if(APPLE)
  add_subdirectory(eigen_accelerate_nested)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
   TARGET MKL::MKL AND
   PGO_MKL_THREADING STREQUAL "tbb_thread")
  add_subdirectory(eigen_mkl_crossover)
  add_subdirectory(eigen_mkl_control_matrix)
  add_subdirectory(eigen_mkl_nested)
  add_subdirectory(eigen_mkl_task_profile)
endif()
