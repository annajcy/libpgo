find_package(Python REQUIRED COMPONENTS Interpreter Development.Module Development.SABIModule)

set(PYPGO_BINDING_SOURCES
  core.cpp
  module.cpp

  # sparse / dense
  sparse/bindings.cpp
  dense/core.cpp
  dense/bindings.cpp

  # mesh
  mesh/geo/core.cpp
  mesh/geo/bindings.cpp
  mesh/volume/core.cpp
  mesh/volume/bindings.cpp

  # fem
  fem/mass/core.cpp
  fem/mass/bindings.cpp
  fem/formulation/bindings.cpp
  fem/formulation/core.cpp
  fem/elastic/bindings.cpp
  fem/elastic/core.cpp
  fem/plastic/bindings.cpp
  fem/plastic/core.cpp

  # energy
  energy/peer.cpp
  energy/core.cpp
  energy/bindings.cpp

  # constraints / solver
  constraints/core.cpp
  constraints/bindings.cpp
  solver/core.cpp
  solver/bindings.cpp

  # simulation
  simulation/dynamic/core.cpp
  simulation/bindings.cpp

  # contact / implicit / parallel
  contact/core.cpp
  contact/bindings.cpp
  implicit/core.cpp
  implicit/bindings.cpp
  parallel/core.cpp
  parallel/bindings.cpp
)

if(TARGET cgalInterface)
  list(APPEND PYPGO_BINDING_SOURCES mesh/remesh_cgal.cpp)
else()
  list(APPEND PYPGO_BINDING_SOURCES mesh/remesh_cgal_disabled.cpp)
endif()

if(TARGET animationIO)
  list(APPEND PYPGO_BINDING_SOURCES
    animation/core.cpp
    animation/bindings.cpp)
else()
  list(APPEND PYPGO_BINDING_SOURCES
    animation/disabled/core.cpp
    animation/disabled/bindings.cpp)
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
    ${CMAKE_CURRENT_SOURCE_DIR}
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
