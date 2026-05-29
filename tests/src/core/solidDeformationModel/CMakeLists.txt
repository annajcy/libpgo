add_executable(simulationMesh_gtest simulationMesh_gtest.cpp)
target_link_libraries(simulationMesh_gtest PRIVATE GTest::gtest_main solidDeformationModel volumetricMesh)
target_compile_definitions(simulationMesh_gtest PRIVATE
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box/box.veg"
  LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/torus.veg")
set_property(TARGET simulationMesh_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(simulationMesh_gtest)

add_executable(deformationModelAssembler_gtest deformationModelAssembler_gtest.cpp)
target_link_libraries(deformationModelAssembler_gtest PRIVATE GTest::gtest_main solidDeformationModel volumetricMesh mesh)
target_compile_definitions(deformationModelAssembler_gtest PRIVATE
  LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/torus.veg"
  LIBPGO_TEST_SHELL_OBJ="${CMAKE_SOURCE_DIR}/tests/fixtures/shell/shell.obj"
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box/box.veg")
set_property(TARGET deformationModelAssembler_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(deformationModelAssembler_gtest)

add_executable(deformationModelFactory_gtest deformationModelFactory_gtest.cpp)
target_link_libraries(deformationModelFactory_gtest PRIVATE GTest::gtest_main solidDeformationModel volumetricMesh mesh)
target_compile_definitions(deformationModelFactory_gtest PRIVATE
  LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/torus.veg"
  LIBPGO_TEST_SHELL_OBJ="${CMAKE_SOURCE_DIR}/tests/fixtures/shell/shell.obj"
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box/box.veg")
set_property(TARGET deformationModelFactory_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(deformationModelFactory_gtest)


add_executable(tetP1Basis_gtest formulations/basis/tetP1Basis_gtest.cpp)
target_link_libraries(tetP1Basis_gtest PRIVATE GTest::gtest_main solidDeformationModel)
set_property(TARGET tetP1Basis_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(tetP1Basis_gtest)

add_executable(hexTrilinearBasis_gtest formulations/basis/hexTrilinearBasis_gtest.cpp)
target_link_libraries(hexTrilinearBasis_gtest PRIVATE GTest::gtest_main solidDeformationModel)
set_property(TARGET hexTrilinearBasis_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(hexTrilinearBasis_gtest)

add_executable(deformationGradientKernel_gtest formulations/kernels/deformationGradientKernel_gtest.cpp)
target_link_libraries(deformationGradientKernel_gtest PRIVATE GTest::gtest_main solidDeformationModel)
set_property(TARGET deformationGradientKernel_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(deformationGradientKernel_gtest)

add_executable(deformationGradientElementModel_gtest formulations/elements/deformationGradientElementModel_gtest.cpp)
target_link_libraries(deformationGradientElementModel_gtest PRIVATE GTest::gtest_main solidDeformationModel)
set_property(TARGET deformationGradientElementModel_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(deformationGradientElementModel_gtest)

add_executable(koiterShellElementModel_gtest formulations/elements/koiterShellElementModel_gtest.cpp)
target_link_libraries(koiterShellElementModel_gtest PRIVATE GTest::gtest_main solidDeformationModel)
set_property(TARGET koiterShellElementModel_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(koiterShellElementModel_gtest)

add_executable(deformationModelFormulation_gtest formulations/deformationModelFormulation_gtest.cpp)
target_link_libraries(deformationModelFormulation_gtest PRIVATE GTest::gtest_main solidDeformationModel volumetricMesh)
target_compile_definitions(deformationModelFormulation_gtest PRIVATE
  LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/torus.veg"
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box/box.veg")
set_property(TARGET deformationModelFormulation_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(deformationModelFormulation_gtest)

add_executable(deformationModelEnergyMaxStep_gtest deformationModelEnergyMaxStep_gtest.cpp)
target_link_libraries(deformationModelEnergyMaxStep_gtest PRIVATE GTest::gtest_main solidDeformationModel simulation mesh)
target_compile_definitions(deformationModelEnergyMaxStep_gtest PRIVATE
  LIBPGO_TEST_SHELL_OBJ="${CMAKE_SOURCE_DIR}/tests/fixtures/shell/shell.obj")
set_property(TARGET deformationModelEnergyMaxStep_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(deformationModelEnergyMaxStep_gtest)

add_executable(elasticModelFactory_gtest factories/elasticModelFactory_gtest.cpp)
target_link_libraries(elasticModelFactory_gtest PRIVATE GTest::gtest_main solidDeformationModel volumetricMesh)
target_compile_definitions(elasticModelFactory_gtest PRIVATE
  LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/torus.veg"
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box/box.veg")
set_property(TARGET elasticModelFactory_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(elasticModelFactory_gtest)

add_executable(elementModelFactory_gtest factories/elementModelFactory_gtest.cpp)
target_link_libraries(elementModelFactory_gtest PRIVATE GTest::gtest_main solidDeformationModel volumetricMesh mesh)
target_compile_definitions(elementModelFactory_gtest PRIVATE
  LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/torus.veg"
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box/box.veg"
  LIBPGO_TEST_SHELL_OBJ="${CMAKE_SOURCE_DIR}/tests/fixtures/shell/shell.obj")
set_property(TARGET elementModelFactory_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(elementModelFactory_gtest)
