set(AVAILABLE_LIBS)

foreach(lib ${PGO_GLOBAL_LIBRARY_TARGETS})
  if(TARGET ${lib})
    list(APPEND AVAILABLE_LIBS ${lib})
  endif()
endforeach()

add_libpgo_tools(core_energy_material_binding_test simulationMesh_material_binding_test.cpp "${AVAILABLE_LIBS}")
target_link_libraries(core_energy_material_binding_test PRIVATE GTest::gtest_main)
set_property(TARGET core_energy_material_binding_test PROPERTY FOLDER "tests")

add_test(NAME core.energy.material_binding COMMAND core_energy_material_binding_test)
