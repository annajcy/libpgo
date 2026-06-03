add_executable(configFileJSON_gtest configFileJSON_gtest.cpp)
target_link_libraries(configFileJSON_gtest PRIVATE GTest::gtest_main configFileJSON)
target_compile_definitions(configFileJSON_gtest PRIVATE
  LIBPGO_TEST_BOX_JSON="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/box/box.json"
  LIBPGO_TEST_CUBIC_BOX_JSON="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box/box.json"
  LIBPGO_TEST_SHELL_JSON="${CMAKE_SOURCE_DIR}/tests/fixtures/config/shell_paths.json")
set_property(TARGET configFileJSON_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(configFileJSON_gtest)

add_executable(parallelism_gtest parallelism_gtest.cpp)
target_link_libraries(parallelism_gtest PRIVATE GTest::gtest_main parallelism)
set_property(TARGET parallelism_gtest PROPERTY FOLDER "tests/gtest")
add_test(NAME parallelism_gtest COMMAND parallelism_gtest)

if (TARGET animationIO)
  add_executable(animationLoader_gtest animationLoader_gtest.cpp)
  target_link_libraries(animationLoader_gtest PRIVATE GTest::gtest_main animationIO)
  set_property(TARGET animationLoader_gtest PROPERTY FOLDER "tests/gtest")
  pgo_gtest_discover_tests(animationLoader_gtest)
endif()

if(TARGET nonlinearOptimization)
  add_executable(NewtonSolver_gtest solver/NewtonSolver_gtest.cpp)
  target_link_libraries(NewtonSolver_gtest PRIVATE GTest::gtest_main nonlinearOptimization)
  set_property(TARGET NewtonSolver_gtest PROPERTY FOLDER "tests/gtest")
  pgo_gtest_discover_tests(NewtonSolver_gtest)

  add_executable(optimizationProblem_gtest solver/optimizationProblem_gtest.cpp)
  target_link_libraries(optimizationProblem_gtest PRIVATE GTest::gtest_main nonlinearOptimization)
  set_property(TARGET optimizationProblem_gtest PROPERTY FOLDER "tests/gtest")
  pgo_gtest_discover_tests(optimizationProblem_gtest)

  add_executable(newtonOptimizer_gtest solver/newtonOptimizer_gtest.cpp)
  target_link_libraries(newtonOptimizer_gtest PRIVATE GTest::gtest_main nonlinearOptimization pgoLogging)
  set_property(TARGET newtonOptimizer_gtest PROPERTY FOLDER "tests/gtest")
  pgo_gtest_discover_tests(newtonOptimizer_gtest)

  add_executable(evaluation_gtest evaluation_gtest.cpp)
  target_link_libraries(evaluation_gtest PRIVATE GTest::gtest_main nonlinearOptimization)
  set_property(TARGET evaluation_gtest PROPERTY FOLDER "tests/gtest")
  pgo_gtest_discover_tests(evaluation_gtest)

  add_executable(linearConstraintFunctions_ownership_gtest linearConstraintFunctions_ownership_gtest.cpp)
  target_link_libraries(linearConstraintFunctions_ownership_gtest PRIVATE GTest::gtest_main nonlinearOptimization)
  set_property(TARGET linearConstraintFunctions_ownership_gtest PROPERTY FOLDER "tests/gtest")
  pgo_gtest_discover_tests(linearConstraintFunctions_ownership_gtest)

  add_executable(constraintEvaluation_gtest constraintEvaluation_gtest.cpp)
  target_link_libraries(constraintEvaluation_gtest PRIVATE GTest::gtest_main nonlinearOptimization)
  set_property(TARGET constraintEvaluation_gtest PROPERTY FOLDER "tests/gtest")
  pgo_gtest_discover_tests(constraintEvaluation_gtest)
endif()

if(TARGET genericPotentialEnergies)
  add_executable(linearPotentialEnergy_ownership_gtest linearPotentialEnergy_ownership_gtest.cpp)
  target_link_libraries(linearPotentialEnergy_ownership_gtest PRIVATE GTest::gtest_main genericPotentialEnergies)
  set_property(TARGET linearPotentialEnergy_ownership_gtest PROPERTY FOLDER "tests/gtest")
  pgo_gtest_discover_tests(linearPotentialEnergy_ownership_gtest)

  add_executable(quadraticPotentialEnergy_ownership_gtest quadraticPotentialEnergy_ownership_gtest.cpp)
  target_link_libraries(quadraticPotentialEnergy_ownership_gtest PRIVATE GTest::gtest_main genericPotentialEnergies)
  set_property(TARGET quadraticPotentialEnergy_ownership_gtest PROPERTY FOLDER "tests/gtest")
  pgo_gtest_discover_tests(quadraticPotentialEnergy_ownership_gtest)
endif()

if(TARGET simulation)
  add_executable(dynamicStepper_gtest dynamicStepper_gtest.cpp)
  target_link_libraries(dynamicStepper_gtest PRIVATE GTest::gtest_main simulation)
  set_property(TARGET dynamicStepper_gtest PROPERTY FOLDER "tests/gtest")
  pgo_gtest_discover_tests(dynamicStepper_gtest)
endif()

if(TARGET implicitSurface)
  add_subdirectory(implicitSurface)
endif()

add_subdirectory(basicAlgorithms)
add_subdirectory(contact)
add_subdirectory(mesh)
add_subdirectory(profiling)
add_subdirectory(solidDeformationModel)
