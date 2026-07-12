set(VOLUMETRIC_MESH_H
  cubicMesh.h
  tetMesh.h
  volumetricMeshENuMaterial.h
  volumetricMesh.h
  volumetricMeshMooneyRivlinMaterial.h
  volumetricMeshOrthotropicMaterial.h
  volumetricMeshParser.h
  vegFile.h
  generateMassMatrix.h
  generateSurfaceMesh.h
)
set(VOLUMETRIC_MESH_SRC
  cubicMesh.cpp
  tetMesh.cpp
  volumetricMesh.cpp
  volumetricMeshENuMaterial.cpp
  volumetricMeshMooneyRivlinMaterial.cpp
  volumetricMeshOrthotropicMaterial.cpp
  volumetricMeshParser.cpp
  vegFile.cpp
  generateMassMatrix.cpp
  generateSurfaceMesh.cpp
)

pgo_require_targets(volumetricMesh TARGETS mesh parallelism TBB::tbb)

if(TARGET Gmsh::Gmsh)
  list(APPEND VOLUMETRIC_MESH_SRC loadMshFile.cpp)
  list(APPEND VOLUMETRIC_MESH_H loadMshFile.h)
  message(STATUS "Including loadMshFile.cpp")
else()
  message(STATUS "Gmsh not found. loadMshFile.cpp will not be included.")
endif()

add_libpgo_lib(volumetricMesh "${VOLUMETRIC_MESH_SRC}" "${VOLUMETRIC_MESH_H}")

target_link_libraries(volumetricMesh PUBLIC mesh)
target_link_libraries(volumetricMesh PRIVATE parallelism TBB::tbb)

if(TARGET Gmsh::Gmsh)
  target_link_libraries(volumetricMesh PUBLIC Gmsh::Gmsh)
  target_compile_definitions(volumetricMesh PUBLIC PGO_HAS_GMSH=1)
endif()
