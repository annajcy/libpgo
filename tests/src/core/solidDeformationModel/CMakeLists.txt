pgo_add_gtest(simulationMesh_gtest
  SOURCES simulationMesh_gtest.cpp
  DEPS solidDeformationModel volumetricMesh
  DEFINITIONS
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/examples/assets/veg/cubic/box.veg"
  LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/examples/assets/veg/tet/torus.veg"
)

pgo_add_gtest(deformationModelAssembler_gtest
  SOURCES deformationModelAssembler_gtest.cpp
  DEPS solidDeformationModel volumetricMesh mesh
  DEFINITIONS
  LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/examples/assets/veg/tet/torus.veg"
  LIBPGO_TEST_SHELL_OBJ="${CMAKE_SOURCE_DIR}/examples/assets/obj/shell.obj"
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/examples/assets/veg/cubic/box.veg"
)

pgo_add_gtest(deformationModelBuilder_gtest
  SOURCES deformationModelBuilder_gtest.cpp
  DEPS solidDeformationModel volumetricMesh mesh
  DEFINITIONS
  LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/examples/assets/veg/tet/torus.veg"
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/examples/assets/veg/cubic/box.veg"
  LIBPGO_TEST_SHELL_OBJ="${CMAKE_SOURCE_DIR}/examples/assets/obj/shell.obj"
)
pgo_add_gtest(materialState_gtest
  SOURCES materialState_gtest.cpp
  DEPS solidDeformationModel volumetricMesh
  DEFINITIONS LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/examples/assets/veg/cubic/box.veg"
)

pgo_add_gtest(tetLinearShapeFunction_gtest
  SOURCES formulations/shapeFunction/tetLinearShapeFunction_gtest.cpp
  DEPS solidDeformationModel
)
pgo_add_gtest(cubicLinearShapeFunction_gtest
  SOURCES formulations/shapeFunction/cubicLinearShapeFunction_gtest.cpp
  DEPS solidDeformationModel
)
pgo_add_gtest(cubicTricubicHermiteShapeFunction_gtest
  SOURCES formulations/shapeFunction/cubicTricubicHermiteShapeFunction_gtest.cpp
  DEPS solidDeformationModel
)
pgo_add_gtest(gaussLegendreHexQuadrature4_gtest
  SOURCES formulations/quadrature/gaussLegendreHexQuadrature4_gtest.cpp
  DEPS solidDeformationModel
)
pgo_add_gtest(cubicTricubicHermiteFormulation_gtest
  SOURCES formulations/cubicTricubicHermiteFormulation_gtest.cpp
  DEPS solidDeformationModel mesh
)
pgo_add_gtest(formulationDynamics_gtest
  SOURCES formulations/formulationDynamics_gtest.cpp
  DEPS solidDeformationModel volumetricMesh mesh
)
pgo_add_gtest(volumetricDeformationElement_gtest
  SOURCES deformation/volume/volumetricDeformationElement_gtest.cpp
  DEPS solidDeformationModel
)
pgo_add_gtest(shellDeformationElement_gtest
  SOURCES deformation/shell/shellDeformationElement_gtest.cpp
  DEPS solidDeformationModel
)

pgo_add_gtest(deformationModelFormulation_gtest
  SOURCES formulations/deformationModelFormulation_gtest.cpp
  DEPS solidDeformationModel volumetricMesh
  DEFINITIONS
  LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/examples/assets/veg/tet/torus.veg"
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/examples/assets/veg/cubic/box.veg"
)

pgo_add_gtest(deformationModelEnergyMaxStep_gtest
  SOURCES deformationModelEnergyMaxStep_gtest.cpp
  DEPS solidDeformationModel simulation mesh
  DEFINITIONS LIBPGO_TEST_SHELL_OBJ="${CMAKE_SOURCE_DIR}/examples/assets/obj/shell.obj"
)

pgo_add_gtest(vertex3DofLayout_gtest
  SOURCES formulations/dof/vertex3DofLayout_gtest.cpp
  DEPS solidDeformationModel mesh
  DEFINITIONS LIBPGO_TEST_SHELL_OBJ="${CMAKE_SOURCE_DIR}/examples/assets/obj/shell.obj"
)

pgo_add_gtest(materialFrames_gtest
  SOURCES material/fields/materialFrames_gtest.cpp
  DEPS solidDeformationModel
)

pgo_add_gtest(modelDefinition_gtest
  SOURCES modelDefinition_gtest.cpp
  DEPS solidDeformationModel volumetricMesh
  DEFINITIONS
  LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/examples/assets/veg/tet/torus.veg"
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/examples/assets/veg/cubic/box.veg"
)

pgo_add_gtest(isotropicSpectralTangent_gtest
  SOURCES material/elastic/deformationGradient/spectral/isotropicSpectralTangent_gtest.cpp
  DEPS solidDeformationModel
)

pgo_add_gtest(elasticModel3DValanisLandel_gtest
  SOURCES material/elastic/deformationGradient/spectral/elasticModel3DValanisLandel_gtest.cpp
  DEPS solidDeformationModel
)

pgo_add_gtest(elasticModel3DSystematicPoking_gtest
  SOURCES material/elastic/deformationGradient/spectral/elasticModel3DSystematicPoking_gtest.cpp
  DEPS solidDeformationModel
)

pgo_add_gtest(elasticModel3DSystematicPokingNeoHookeanComparison_gtest
  SOURCES material/elastic/deformationGradient/spectral/elasticModel3DSystematicPokingNeoHookeanComparison_gtest.cpp
  DEPS solidDeformationModel
)

pgo_add_gtest(elasticModel3DNeoHookean_gtest
  SOURCES material/elastic/elasticModel3DNeoHookean_gtest.cpp
  DEPS solidDeformationModel
)

pgo_add_gtest(elasticModel1D_gtest
  SOURCES material/elastic/elasticModel1D_gtest.cpp
  DEPS solidDeformationModel
)

pgo_add_gtest(integratedLinearCurvatureSpline_gtest
  SOURCES material/elastic/spline/integratedLinearCurvatureSpline_gtest.cpp
  DEPS solidDeformationModel
)
