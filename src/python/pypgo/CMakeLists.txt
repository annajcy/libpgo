find_package(Python REQUIRED COMPONENTS Interpreter Development.Module Development.SABIModule)

set(PYPGO_BINDING_SOURCES
  bindings/module.cpp

  # sparse / dense
  bindings/sparse/bindings.cpp
  bindings/dense/bindings.cpp

  # mesh
  bindings/mesh/geo_bindings.cpp
  bindings/mesh/volume_bindings.cpp

  # fem
  bindings/fem/formulation_bindings.cpp
  bindings/fem/formulation_core.cpp
  bindings/fem/elastic_bindings.cpp
  bindings/fem/elastic_core.cpp
  bindings/fem/plastic_bindings.cpp
  bindings/fem/plastic_core.cpp

  # energy
  bindings/energy/bindings.cpp

  # constraints / solver
  bindings/constraints/constraint_bindings.cpp
  bindings/solver/solver_bindings.cpp

  # simulation
  bindings/simulation/bindings.cpp

  # contact / implicit / parallel
  bindings/contact/bindings.cpp
  bindings/implicit/bindings.cpp
  bindings/parallel/bindings.cpp
)

if(TARGET cgalInterface)
  list(APPEND PYPGO_BINDING_SOURCES bindings/mesh/remesh_cgal.cpp)
else()
  list(APPEND PYPGO_BINDING_SOURCES bindings/mesh/remesh_cgal_disabled.cpp)
endif()

if(TARGET animationIO)
  list(APPEND PYPGO_BINDING_SOURCES bindings/animation/animation_io.cpp)
else()
  list(APPEND PYPGO_BINDING_SOURCES bindings/animation/animation_io_disabled.cpp)
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
  simulation
  contact
  tetgenInterface
  volumetricMeshMeshing
  implicitSurface
  parallelism
)

if(TARGET cgalInterface)
  target_link_libraries(pypgo_core PRIVATE cgalInterface)
  target_include_directories(pypgo_core PRIVATE ${CMAKE_SOURCE_DIR}/src/core/cgalInterface)
  target_compile_definitions(pypgo_core PRIVATE PYPGO_HAS_CGAL=1)
endif()

if(TARGET animationIO)
  target_link_libraries(pypgo_core PRIVATE animationIO)
  target_include_directories(pypgo_core PRIVATE ${CMAKE_SOURCE_DIR}/src/core/animationIO)
  if(TARGET openvdb_static OR TARGET OpenVDB::openvdb)
    target_compile_definitions(pypgo_core PRIVATE PYPGO_HAS_STRESS_VDB=1)
  endif()
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
    ${CMAKE_SOURCE_DIR}/src/core/simulation
    ${CMAKE_SOURCE_DIR}/src/core/contact
    ${CMAKE_SOURCE_DIR}/src/core/tetgenInterface
    ${CMAKE_SOURCE_DIR}/src/core/volumetricMeshMeshing
    ${CMAKE_SOURCE_DIR}/src/core/implicitSurface
    ${CMAKE_SOURCE_DIR}/src/core/parallelism
)

set_target_properties(pypgo_core PROPERTIES
  OUTPUT_NAME "_core"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/pypgo"
)
