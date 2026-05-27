find_package(Python REQUIRED COMPONENTS Development.Module)

set(PYPGO_BINDING_SOURCES
  bindings/module.cpp
  bindings/dense_bindings.cpp
  bindings/mesh_geo_bindings.cpp
  bindings/mesh_bindings.cpp
  bindings/sparse_bindings.cpp
  ../../tools/mesh/volumetric/cubicMesher/triangleMeshVoxelizer.cpp
  ../../tools/mesh/volumetric/tetMesher/tetMesherBackend.cpp
  ../../tools/mesh/volumetric/tetMesher/tetgenBackend.cpp
)

if(PGO_TET_MESHER_USE_TET_WILD)
  list(APPEND PYPGO_BINDING_SOURCES ../../tools/mesh/volumetric/tetMesher/tetwildBackend.cpp)
else()
  list(APPEND PYPGO_BINDING_SOURCES ../../tools/mesh/volumetric/tetMesher/tetwildBackendDisabled.cpp)
endif()

nanobind_add_module(pypgo_core ${PYPGO_BINDING_SOURCES})

target_link_libraries(pypgo_core PRIVATE
  mesh
  volumetricMesh
  interpolationCoordinates
  solidDeformationModel
  tetgenInterface
)

if(PGO_TET_MESHER_USE_TET_WILD)
  target_link_libraries(pypgo_core PRIVATE FloatTetwild)
  target_compile_definitions(pypgo_core PRIVATE PGO_TET_MESHER_HAS_TET_WILD=1)
endif()

target_include_directories(pypgo_core
  PRIVATE
    ${CMAKE_SOURCE_DIR}/src/core/mesh
    ${CMAKE_SOURCE_DIR}/src/core/volumetricMesh
    ${CMAKE_SOURCE_DIR}/src/core/interpolationCoordinates
    ${CMAKE_SOURCE_DIR}/src/core/eigenSupport
    ${CMAKE_SOURCE_DIR}/src/core/solidDeformationModel
    ${CMAKE_SOURCE_DIR}/src/core/tetgenInterface
    ${CMAKE_SOURCE_DIR}/src/tools/mesh/volumetric/cubicMesher
    ${CMAKE_SOURCE_DIR}/src/tools/mesh/volumetric/tetMesher
)

set_target_properties(pypgo_core PROPERTIES
  OUTPUT_NAME "_core"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/pypgo"
)
