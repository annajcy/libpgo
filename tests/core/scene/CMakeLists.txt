set(AVAILABLE_LIBS)

foreach(lib ${PGO_GLOBAL_LIBRARY_TARGETS})
  if(TARGET ${lib})
    list(APPEND AVAILABLE_LIBS ${lib})
  endif()
endforeach()

set(CORE_SCENE_GEOMETRY_SOURCES
  geometry_test.cpp
)

set(CORE_SCENE_VOLUMETRIC_MESH_SOURCES
  volumetricMesh/tet_mesh_test.cpp
  volumetricMesh/cubic_mesh_test.cpp
  volumetricMesh/algorithms/cubic_mesh_interpolation_test.cpp
  volumetricMesh/algorithms/cubic_mesh_normal_correction_test.cpp
  volumetricMesh/algorithms/cubic_mesh_refinement_test.cpp
  volumetricMesh/ops/tet_mesh_ops_test.cpp
  volumetricMesh/ops/cubic_mesh_ops_test.cpp
  volumetricMesh/mesh_queries_test.cpp
  volumetricMesh/volumetric_mesh_edit_test.cpp
  volumetricMesh/generate_surface_mesh_test.cpp
  volumetricMesh/volumetric_mesh_clone_test.cpp
  volumetricMesh/io/mesh_format_detector_test.cpp
  volumetricMesh/io/mesh_ascii_io_test.cpp
  volumetricMesh/io/mesh_binary_io_test.cpp
  volumetricMesh/io/material_serde_test.cpp
  volumetricMesh/io/mesh_parser_include_test.cpp
  volumetricMesh/internal/material_catalog_test.cpp
  volumetricMesh/internal/mesh_subset_editor_test.cpp
  volumetricMesh/internal/volumetric_mesh_data_test.cpp
)

add_libpgo_tools(core_scene_geometry_test "${CORE_SCENE_GEOMETRY_SOURCES}" "${AVAILABLE_LIBS}")
target_link_libraries(core_scene_geometry_test PRIVATE GTest::gtest_main)
set_property(TARGET core_scene_geometry_test PROPERTY FOLDER "tests")

add_test(NAME core.scene.geometry COMMAND core_scene_geometry_test)

add_libpgo_tools(core_scene_volumetric_mesh_test "${CORE_SCENE_VOLUMETRIC_MESH_SOURCES}" "${AVAILABLE_LIBS}")
target_link_libraries(core_scene_volumetric_mesh_test PRIVATE GTest::gtest_main)
set_property(TARGET core_scene_volumetric_mesh_test PROPERTY FOLDER "tests")

add_test(NAME core.scene.volumetric_mesh COMMAND core_scene_volumetric_mesh_test)
