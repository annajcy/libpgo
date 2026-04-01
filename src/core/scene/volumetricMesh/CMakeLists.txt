set(VOLUMETRIC_MESH_H
  algorithms/cubic_mesh_interpolation.h
  algorithms/cubic_mesh_normal_correction.h
  algorithms/cubic_mesh_refinement.h
  cubicMesh.h
  io/detail/tetgen_reader.h
  io/detail/veg_parser.h
  io/mesh_ascii_reader.h
  io/mesh_ascii_writer.h
  io/mesh_binary_reader.h
  io/mesh_binary_writer.h
  io/mesh_loaders.h
  io/material_serde.h
  io/mesh_format_detector.h
  io/mesh_io_types.h
  internal/material_catalog.h
  internal/mesh_mass_properties.h
  internal/mesh_mutation.h
  internal/mesh_queries.h
  internal/mesh_subset_editor.h
  internal/mesh_transforms.h
  internal/region_assignment.h
  internal/volumetric_mesh_data.h
  ops/cubic_mesh_ops.h
  ops/tet_mesh_ops.h
  tetMesh.h
  volumetricMeshENuMaterial.h
  volumetricMesh.h
  volumetricMeshEdit.h
  volumetricMeshExport.h
  volumetricMeshIO.h
  volumetricMeshInterpolation.h
  volumetricMeshMooneyRivlinMaterial.h
  volumetricMeshOrthotropicMaterial.h
  volumetricMeshTypes.h
  generateMassMatrix.h
  generateSurfaceMesh.h
)
set(VOLUMETRIC_MESH_SRC
  algorithms/cubic_mesh_interpolation.cpp
  algorithms/cubic_mesh_normal_correction.cpp
  algorithms/cubic_mesh_refinement.cpp
  cubicMesh.cpp
  io/detail/tetgen_reader.cpp
  io/detail/veg_parser.cpp
  io/mesh_ascii_reader.cpp
  io/mesh_ascii_writer.cpp
  io/mesh_binary_reader.cpp
  io/mesh_binary_writer.cpp
  io/material_serde.cpp
  io/mesh_format_detector.cpp
  internal/material_catalog.cpp
  internal/mesh_mass_properties.cpp
  internal/mesh_mutation.cpp
  internal/mesh_queries.cpp
  internal/mesh_subset_editor.cpp
  internal/mesh_transforms.cpp
  internal/region_assignment.cpp
  internal/volumetric_mesh_data.cpp
  ops/cubic_mesh_ops.cpp
  ops/tet_mesh_ops.cpp
  tetMesh.cpp
  volumetricMesh.cpp
  volumetricMeshEdit.cpp
  volumetricMeshENuMaterial.cpp
  volumetricMeshExport.cpp
  volumetricMeshIO.cpp
  volumetricMeshInterpolation.cpp
  volumetricMeshMooneyRivlinMaterial.cpp
  volumetricMeshOrthotropicMaterial.cpp
  generateMassMatrix.cpp
  generateSurfaceMesh.cpp
)

if(NOT TARGET mesh)
  message(STATUS "volumetricMesh is not included. Missing: mesh")
  return()
endif()

if(TARGET Gmsh::Gmsh)
  list(APPEND VOLUMETRIC_MESH_SRC loadMshFile.cpp)
  list(APPEND VOLUMETRIC_MESH_H loadMshFile.h)
  message(STATUS "Including loadMshFile.cpp")
else()
  message(STATUS "Gmsh not found. loadMshFile.cpp will not be included.")
endif()

add_libpgo_lib(volumetricMesh "${VOLUMETRIC_MESH_SRC}" "${VOLUMETRIC_MESH_H}")

target_link_libraries(volumetricMesh PUBLIC mesh)

if(TARGET Gmsh::Gmsh)
  target_link_libraries(volumetricMesh PUBLIC Gmsh::Gmsh)
endif()
