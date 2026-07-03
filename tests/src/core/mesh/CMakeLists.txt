pgo_add_gtest(triMeshGeo_gtest
  SOURCES triMeshGeo_gtest.cpp
  DEPS mesh
)
pgo_add_gtest(meshData_gtest
  SOURCES meshData_gtest.cpp
  DEPS mesh
)
pgo_add_gtest(tetMeshGeo_gtest
  SOURCES tetMeshGeo_gtest.cpp
  DEPS mesh volumetricMesh
)
pgo_add_gtest(cubicMeshGeo_gtest
  SOURCES cubicMeshGeo_gtest.cpp
  DEPS mesh volumetricMesh
)
pgo_add_gtest(vegFile_gtest
  SOURCES vegFile_gtest.cpp
  DEPS mesh volumetricMesh
)
pgo_add_gtest(mesh_gtest
  SOURCES mesh_gtest.cpp
  DEPS mesh
)
