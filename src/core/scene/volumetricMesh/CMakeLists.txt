set(VOLUMETRIC_MESH_H
  cubicMesh.h
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
  tetMesh.h
  volumetricMeshENuMaterial.h
  volumetricMesh.h
  volumetricMeshEdit.h
  volumetricMeshExport.h
  volumetricMeshIO.h
  volumetricMeshInterpolation.h
  volumetricMeshMooneyRivlinMaterial.h
  volumetricMeshOrthotropicMaterial.h
  volumetricMeshParser.h
  volumetricMeshTypes.h
  generateMassMatrix.h
  generateSurfaceMesh.h
)
set(VOLUMETRIC_MESH_SRC
  cubicMesh.cpp
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
  tetMesh.cpp
  volumetricMesh.cpp
  volumetricMeshEdit.cpp
  volumetricMeshENuMaterial.cpp
  volumetricMeshExport.cpp
  volumetricMeshIO.cpp
  volumetricMeshInterpolation.cpp
  volumetricMeshMooneyRivlinMaterial.cpp
  volumetricMeshOrthotropicMaterial.cpp
  volumetricMeshParser.cpp
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
