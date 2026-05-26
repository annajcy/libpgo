find_package(Python REQUIRED COMPONENTS Development.Module)

set(PYPGO_BINDING_SOURCES
  bindings/module.cpp
  bindings/mesh_geo_bindings.cpp
  bindings/mesh_bindings.cpp
)

nanobind_add_module(pypgo_core ${PYPGO_BINDING_SOURCES})

target_link_libraries(pypgo_core PRIVATE mesh volumetricMesh)

target_include_directories(pypgo_core
  PRIVATE
    ${CMAKE_SOURCE_DIR}/src/core/mesh
    ${CMAKE_SOURCE_DIR}/src/core/volumetricMesh
)

set_target_properties(pypgo_core PROPERTIES
  OUTPUT_NAME "_core"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/pypgo"
)
