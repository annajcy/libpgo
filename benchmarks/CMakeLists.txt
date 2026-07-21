function(pgo_add_mkl_tbb_probe target)
  add_executable(${target} ${ARGN})
  target_link_libraries(${target}
    PRIVATE
      Eigen3::Eigen
      MKL::MKL
      parallel
      TBB::tbb
  )
endfunction()

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
  add_subdirectory(eigen_mkl_budget_sweep)
  add_subdirectory(eigen_mkl_control_matrix)
  add_subdirectory(eigen_mkl_nested_inner_arena)
  add_subdirectory(eigen_mkl_task_profile)
  add_subdirectory(cubic_linear_pardiso_aftermath)
endif()
