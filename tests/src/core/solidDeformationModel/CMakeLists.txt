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

add_executable(deformationModelState_gtest deformationModelState_gtest.cpp)
target_link_libraries(deformationModelState_gtest PRIVATE GTest::gtest_main solidDeformationModel volumetricMesh)
target_compile_definitions(deformationModelState_gtest PRIVATE
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box/box.veg")
set_property(TARGET deformationModelState_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(deformationModelState_gtest)

add_executable(tetP1Basis_gtest formulations/basis/tetP1Basis_gtest.cpp)
target_link_libraries(tetP1Basis_gtest PRIVATE GTest::gtest_main solidDeformationModel)
set_property(TARGET tetP1Basis_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(tetP1Basis_gtest)

add_executable(hexTrilinearBasis_gtest formulations/basis/hexTrilinearBasis_gtest.cpp)
target_link_libraries(hexTrilinearBasis_gtest PRIVATE GTest::gtest_main solidDeformationModel)
set_property(TARGET hexTrilinearBasis_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(hexTrilinearBasis_gtest)

add_executable(volumetricKernel_gtest formulations/kernels/volumetricKernel_gtest.cpp)
target_link_libraries(volumetricKernel_gtest PRIVATE GTest::gtest_main solidDeformationModel)
set_property(TARGET volumetricKernel_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(volumetricKernel_gtest)

add_executable(volumetricDeformationModel_gtest formulations/elements/volumetricDeformationModel_gtest.cpp)
target_link_libraries(volumetricDeformationModel_gtest PRIVATE GTest::gtest_main solidDeformationModel)
set_property(TARGET volumetricDeformationModel_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(volumetricDeformationModel_gtest)

add_executable(shellDeformationModel_gtest formulations/elements/shellDeformationModel_gtest.cpp)
target_link_libraries(shellDeformationModel_gtest PRIVATE GTest::gtest_main solidDeformationModel)
set_property(TARGET shellDeformationModel_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(shellDeformationModel_gtest)

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

add_executable(vertex3DofLayout_gtest
  formulations/dof/vertex3DofLayout_gtest.cpp)
target_link_libraries(vertex3DofLayout_gtest PRIVATE GTest::gtest_main solidDeformationModel mesh)
target_compile_definitions(vertex3DofLayout_gtest PRIVATE
  LIBPGO_TEST_SHELL_OBJ="${CMAKE_SOURCE_DIR}/tests/fixtures/shell/shell.obj")
set_property(TARGET vertex3DofLayout_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(vertex3DofLayout_gtest)

add_executable(constantParameterField_gtest formulations/parameters/constantParameterField_gtest.cpp)
target_link_libraries(constantParameterField_gtest PRIVATE GTest::gtest_main solidDeformationModel)
set_property(TARGET constantParameterField_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(constantParameterField_gtest)

add_executable(elementwiseParameterField_gtest formulations/parameters/elementwiseParameterField_gtest.cpp)
target_link_libraries(elementwiseParameterField_gtest PRIVATE GTest::gtest_main solidDeformationModel)
set_property(TARGET elementwiseParameterField_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(elementwiseParameterField_gtest)

add_executable(elasticModelFactory_gtest factories/elasticModelFactory_gtest.cpp)
target_link_libraries(elasticModelFactory_gtest PRIVATE GTest::gtest_main solidDeformationModel volumetricMesh)
target_compile_definitions(elasticModelFactory_gtest PRIVATE
  LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/torus.veg"
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box/box.veg")
set_property(TARGET elasticModelFactory_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(elasticModelFactory_gtest)

add_executable(deformationModelFactory_gtest factories/deformationModelFactory_gtest.cpp)
target_link_libraries(deformationModelFactory_gtest PRIVATE GTest::gtest_main solidDeformationModel volumetricMesh mesh)
target_compile_definitions(deformationModelFactory_gtest PRIVATE
  LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/torus.veg"
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box/box.veg"
  LIBPGO_TEST_SHELL_OBJ="${CMAKE_SOURCE_DIR}/tests/fixtures/shell/shell.obj")
set_property(TARGET deformationModelFactory_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(deformationModelFactory_gtest)
