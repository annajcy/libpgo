pgo_add_gtest(parallel_gtest
  SOURCES parallel_gtest.cpp
  DEPS parallel
)

set(PARALLEL_TEST_TARGETS
  parallel_gtest
)

foreach(target IN LISTS PARALLEL_TEST_TARGETS)
  target_link_libraries(${target} PRIVATE TBB::tbb)
  if(TARGET Eigen3::Eigen)
    target_link_libraries(${target} PRIVATE Eigen3::Eigen)
    target_compile_definitions(${target} PRIVATE PGO_PARALLELISM_HAS_EIGEN)
  endif()
  if(TARGET MKL::MKL)
    target_link_libraries(${target} PRIVATE MKL::MKL)
    target_compile_definitions(${target} PRIVATE PGO_HAS_MKL)
    if(PGO_MKL_THREADING STREQUAL "tbb_thread")
      target_compile_definitions(${target} PRIVATE PGO_PARALLELISM_MKL_TBB_THREADING)
    endif()
  endif()
endforeach()

if(TARGET cgalInterface)
  pgo_add_gtest(cgalInterface_gtest
    SOURCES cgalInterface_gtest.cpp
    DEPS cgalInterface
    INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/src/core/cgalInterface ${CMAKE_SOURCE_DIR}/src/core/mesh
  )
endif()

if (TARGET animationIO)
  pgo_add_gtest(animationLoader_gtest
    SOURCES animationLoader_gtest.cpp
    DEPS animationIO
  )
endif()

if(TARGET nonlinearOptimization)
  pgo_add_gtest(NewtonSolver_gtest
    SOURCES solver/NewtonSolver_gtest.cpp
    DEPS nonlinearOptimization
  )
  pgo_add_gtest(optimizationProblem_gtest
    SOURCES solver/optimizationProblem_gtest.cpp
    DEPS nonlinearOptimization
  )
  pgo_add_gtest(newtonOptimizer_gtest
    SOURCES solver/newtonOptimizer_gtest.cpp
    DEPS nonlinearOptimization pgoLogging
  )
  pgo_add_gtest(evaluation_gtest
    SOURCES evaluation_gtest.cpp
    DEPS nonlinearOptimization
  )
  pgo_add_gtest(linearConstraintFunctions_ownership_gtest
    SOURCES linearConstraintFunctions_ownership_gtest.cpp
    DEPS nonlinearOptimization
  )
  pgo_add_gtest(constraintEvaluation_gtest
    SOURCES constraintEvaluation_gtest.cpp
    DEPS nonlinearOptimization
  )
endif()

if(TARGET genericPotentialEnergies)
  pgo_add_gtest(linearPotentialEnergy_ownership_gtest
    SOURCES linearPotentialEnergy_ownership_gtest.cpp
    DEPS genericPotentialEnergies
  )
  pgo_add_gtest(quadraticPotentialEnergy_ownership_gtest
    SOURCES quadraticPotentialEnergy_ownership_gtest.cpp
    DEPS genericPotentialEnergies
  )
endif()

if(TARGET simulation)
  pgo_add_gtest(dynamicStepper_gtest
    SOURCES dynamicStepper_gtest.cpp
    DEPS simulation
  )
endif()

if(TARGET implicitSurface)
  add_subdirectory(implicitSurface)
endif()

add_subdirectory(basicAlgorithms)
add_subdirectory(contact)
add_subdirectory(mesh)
add_subdirectory(profiling)
add_subdirectory(solidDeformationModel)
