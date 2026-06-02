find_package(Python REQUIRED COMPONENTS Interpreter Development.Module Development.SABIModule)

set(PYPGO_BINDING_SOURCES
  bindings/module.cpp
  bindings/dense_bindings.cpp
  bindings/mesh_geo_bindings.cpp
  bindings/mesh_bindings.cpp
  bindings/energy_bindings.cpp
  bindings/constraints/constraint_bindings.cpp
  bindings/solver/solver_bindings.cpp
  bindings/sparse_bindings.cpp
  bindings/implicit_bindings.cpp
  bindings/parallel_bindings.cpp
  ../../tools/mesh/volumetric/cubicMesher/triangleMeshVoxelizer.cpp
  ../../tools/mesh/volumetric/tetMesher/tetMesherBackend.cpp
  ../../tools/mesh/volumetric/tetMesher/tetgenBackend.cpp
)

if(PGO_TET_MESHER_USE_TET_WILD)
  list(APPEND PYPGO_BINDING_SOURCES ../../tools/mesh/volumetric/tetMesher/tetwildBackend.cpp)
else()
  list(APPEND PYPGO_BINDING_SOURCES ../../tools/mesh/volumetric/tetMesher/tetwildBackendDisabled.cpp)
endif()

if(TARGET cgalInterface)
  list(APPEND PYPGO_BINDING_SOURCES bindings/surface_remesh_cgal.cpp)
else()
  list(APPEND PYPGO_BINDING_SOURCES bindings/surface_remesh_cgal_disabled.cpp)
endif()

if(TARGET geogramInterface)
  list(APPEND PYPGO_BINDING_SOURCES bindings/surface_remesh_geogram.cpp)
else()
  list(APPEND PYPGO_BINDING_SOURCES bindings/surface_remesh_geogram_disabled.cpp)
endif()

if(TARGET animationIO)
  list(APPEND PYPGO_BINDING_SOURCES bindings/animation_io.cpp)
else()
  list(APPEND PYPGO_BINDING_SOURCES bindings/animation_io_disabled.cpp)
endif()

nanobind_add_module(pypgo_core STABLE_ABI NB_DOMAIN pypgo ${PYPGO_BINDING_SOURCES})

target_link_libraries(pypgo_core PRIVATE
  mesh
  volumetricMesh
  interpolationCoordinates
  solidDeformationModel
  nonlinearOptimization
  genericPotentialEnergies
  constraintPotentialEnergies
  tetgenInterface
  implicitSurface
  parallelism
)

if(TARGET cgalInterface)
  target_link_libraries(pypgo_core PRIVATE cgalInterface)
  target_include_directories(pypgo_core PRIVATE ${CMAKE_SOURCE_DIR}/src/core/cgalInterface)
  target_compile_definitions(pypgo_core PRIVATE PYPGO_HAS_CGAL=1)
endif()

if(TARGET geogramInterface)
  target_link_libraries(pypgo_core PRIVATE geogramInterface)
  target_include_directories(pypgo_core PRIVATE ${CMAKE_SOURCE_DIR}/src/core/geogramInterface)
  target_compile_definitions(pypgo_core PRIVATE PYPGO_HAS_GEOGRAM=1)
endif()

if(TARGET animationIO)
  target_link_libraries(pypgo_core PRIVATE animationIO)
  target_include_directories(pypgo_core PRIVATE ${CMAKE_SOURCE_DIR}/src/core/animationIO)
  if(TARGET openvdb_static OR TARGET OpenVDB::openvdb)
    target_compile_definitions(pypgo_core PRIVATE PYPGO_HAS_STRESS_VDB=1)
  endif()
endif()

if(PGO_TET_MESHER_USE_TET_WILD)
  target_link_libraries(pypgo_core PRIVATE FloatTetwild)
  target_compile_definitions(pypgo_core PRIVATE PGO_TET_MESHER_HAS_TET_WILD=1)
endif()

target_include_directories(pypgo_core
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/bindings
    ${CMAKE_SOURCE_DIR}/src/core/mesh
    ${CMAKE_SOURCE_DIR}/src/core/volumetricMesh
    ${CMAKE_SOURCE_DIR}/src/core/interpolationCoordinates
    ${CMAKE_SOURCE_DIR}/src/core/eigenSupport
    ${CMAKE_SOURCE_DIR}/src/core/solidDeformationModel
    ${CMAKE_SOURCE_DIR}/src/core/nonlinearOptimization
    ${CMAKE_SOURCE_DIR}/src/core/genericPotentialEnergies
    ${CMAKE_SOURCE_DIR}/src/core/constraintPotentialEnergies
    ${CMAKE_SOURCE_DIR}/src/core/tetgenInterface
    ${CMAKE_SOURCE_DIR}/src/core/implicitSurface
    ${CMAKE_SOURCE_DIR}/src/core/parallelism
    ${CMAKE_SOURCE_DIR}/src/tools/mesh/volumetric/cubicMesher
    ${CMAKE_SOURCE_DIR}/src/tools/mesh/volumetric/tetMesher
)

set_target_properties(pypgo_core PROPERTIES
  OUTPUT_NAME "_core"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/pypgo"
)
