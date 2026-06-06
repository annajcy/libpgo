set(VOLUMETRIC_MESH_MESHING_HEADERS
  triangleMeshVoxelizer.h
  tetMesherBackend.h
)

set(VOLUMETRIC_MESH_MESHING_SOURCES
  triangleMeshVoxelizer.cpp
  tetMesherBackend.cpp
  tetgenBackend.cpp
)

set(VOLUMETRIC_MESH_MESHING_DEPS
  mesh
  volumetricMesh
  tetgenInterface
)

foreach(tgt ${VOLUMETRIC_MESH_MESHING_DEPS})
  if(NOT TARGET ${tgt})
    message(STATUS "volumetricMeshMeshing is not included. Missing: ${tgt}")
    return()
  endif()
endforeach()

if(PGO_TET_MESHER_USE_TET_WILD)
  if(NOT TARGET FloatTetwild)
    message(STATUS "volumetricMeshMeshing is not included. Missing: FloatTetwild")
    return()
  endif()

  list(APPEND VOLUMETRIC_MESH_MESHING_SOURCES tetwildBackend.cpp)
else()
  list(APPEND VOLUMETRIC_MESH_MESHING_SOURCES tetwildBackendDisabled.cpp)
endif()

add_libpgo_lib(volumetricMeshMeshing
  "${VOLUMETRIC_MESH_MESHING_SOURCES}"
  "${VOLUMETRIC_MESH_MESHING_HEADERS}"
)

target_link_libraries(volumetricMeshMeshing PUBLIC ${VOLUMETRIC_MESH_MESHING_DEPS})

if(PGO_TET_MESHER_USE_TET_WILD)
  target_link_libraries(volumetricMeshMeshing PUBLIC FloatTetwild)
  target_compile_definitions(volumetricMeshMeshing PUBLIC PGO_TET_MESHER_HAS_TET_WILD=1)
endif()
