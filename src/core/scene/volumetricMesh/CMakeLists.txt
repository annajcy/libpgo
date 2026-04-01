set(VOLUMETRIC_MESH_H
  algorithms/cubic_mesh_interpolation.h
  algorithms/cubic_mesh_normal_correction.h
  algorithms/cubic_mesh_refinement.h
  algorithms/generate_mass_matrix.h
  algorithms/generate_surface_mesh.h
  algorithms/mesh_interpolation.h
  cubicMesh.h
  dispatch/any_mesh_ref.h
  io/detail/gmsh_reader.h
  io/detail/tetgen_reader.h
  io/detail/veg_parser.h
  io/mesh_ascii_reader.h
  io/mesh_ascii_writer.h
  io/mesh_binary_reader.h
  io/mesh_binary_writer.h
  io/mesh_load.h
  io/mesh_loaders.h
  io/material_serde.h
  io/mesh_format_detector.h
  io/mesh_io_types.h
  io/mesh_save.h
  io/interpolation_weight_io.h
  internal/material_catalog.h
  internal/mesh_mass_properties.h
  internal/mesh_mutation.h
  internal/mesh_queries.h
  internal/mesh_subset_editor.h
  internal/mesh_transforms.h
  internal/region_assignment.h
  internal/volumetric_mesh_data.h
  materials/material_access.h
  materials/material_record.h
  ops/cubic_mesh_ops.h
  ops/mesh_edit_ops.h
  ops/element_face_ops.h
  ops/mesh_export_ops.h
  ops/mesh_export_types.h
  ops/mesh_construction.h
  ops/tet_geometry_export.h
  ops/tet_mesh_ops.h
  storage/mesh_storage.h
  tetMesh.h
  traits/mesh_tags.h
  traits/mesh_traits.h
  concepts/mesh_concepts.h
  types/element_set.h
  types/element_types.h
  types/interpolation_weights.h
  types/mesh_enums.h
  types/material_region.h
  volumetricMesh.h
)
set(VOLUMETRIC_MESH_SRC
  algorithms/cubic_mesh_interpolation.cpp
  algorithms/cubic_mesh_normal_correction.cpp
  algorithms/cubic_mesh_refinement.cpp
  algorithms/generate_mass_matrix.cpp
  algorithms/generate_surface_mesh.cpp
  algorithms/mesh_interpolation.cpp
  cubicMesh.cpp
  dispatch/any_mesh_ref.cpp
  io/detail/tetgen_reader.cpp
  io/detail/veg_parser.cpp
  io/mesh_ascii_reader.cpp
  io/mesh_ascii_writer.cpp
  io/mesh_binary_reader.cpp
  io/mesh_binary_writer.cpp
  io/mesh_load.cpp
  io/material_serde.cpp
  io/mesh_format_detector.cpp
  io/interpolation_weight_io.cpp
  io/mesh_save.cpp
  internal/material_catalog.cpp
  internal/mesh_mass_properties.cpp
  internal/mesh_mutation.cpp
  internal/mesh_queries.cpp
  internal/mesh_subset_editor.cpp
  internal/mesh_transforms.cpp
  internal/region_assignment.cpp
  internal/volumetric_mesh_data.cpp
  ops/cubic_mesh_ops.cpp
  ops/mesh_edit_ops.cpp
  ops/element_face_ops.cpp
  ops/mesh_export_ops.cpp
  ops/mesh_construction.cpp
  ops/tet_geometry_export.cpp
  ops/tet_mesh_ops.cpp
  storage/mesh_storage.cpp
  tetMesh.cpp
  volumetricMesh.cpp
)

if(NOT TARGET mesh)
  message(STATUS "volumetricMesh is not included. Missing: mesh")
  return()
endif()

if(TARGET Gmsh::Gmsh)
  list(APPEND VOLUMETRIC_MESH_SRC io/detail/gmsh_reader.cpp)
  message(STATUS "Including io/detail/gmsh_reader.cpp")
else()
  message(STATUS "Gmsh not found. io/detail/gmsh_reader.cpp will not be included.")
endif()

add_libpgo_lib(volumetricMesh "${VOLUMETRIC_MESH_SRC}" "${VOLUMETRIC_MESH_H}")

target_link_libraries(volumetricMesh PUBLIC mesh)

if(TARGET Gmsh::Gmsh)
  target_compile_definitions(volumetricMesh PUBLIC PGO_HAS_GMSH)
  target_link_libraries(volumetricMesh PUBLIC Gmsh::Gmsh)
endif()
