pgo_add_gtest(parallelism_gtest
  SOURCES parallelism_gtest.cpp
  DEPS parallelism
)

pgo_add_gtest(parallelism_runtime_gtest
  SOURCES parallelism_runtime_gtest.cpp
  DEPS parallelism
)

pgo_add_gtest(parallelism_observer_gtest
  SOURCES parallelism_observer_gtest.cpp
  DEPS parallelism
)

pgo_add_gtest(parallelism_pressure_gtest
  SOURCES parallelism_pressure_gtest.cpp
  DEPS parallelism
)

set(PARALLELISM_TEST_TARGETS
  parallelism_gtest
  parallelism_runtime_gtest
  parallelism_observer_gtest
  parallelism_pressure_gtest
)

foreach(target IN LISTS PARALLELISM_TEST_TARGETS)
  if(TARGET TBB::tbb)
    target_link_libraries(${target} PRIVATE TBB::tbb)
    target_compile_definitions(${target} PRIVATE PGO_PARALLELISM_HAS_TBB)
  endif()
  if(TARGET Eigen3::Eigen)
    target_link_libraries(${target} PRIVATE Eigen3::Eigen)
    target_compile_definitions(${target} PRIVATE PGO_PARALLELISM_HAS_EIGEN)
  endif()
  if(TARGET MKL::MKL)
    target_link_libraries(${target} PRIVATE MKL::MKL)
    target_compile_definitions(${target} PRIVATE PGO_PARALLELISM_HAS_MKL)
    if(PGO_MKL_THREADING STREQUAL "tbb_thread")
      target_compile_definitions(${target} PRIVATE PGO_PARALLELISM_MKL_TBB_THREADING)
    endif()
  endif()
  if(APPLE)
    target_link_libraries(${target} PRIVATE "-framework Accelerate")
    target_compile_definitions(${target} PRIVATE PGO_PARALLELISM_HAS_ACCELERATE_THREADING)
  endif()
endforeach()

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
