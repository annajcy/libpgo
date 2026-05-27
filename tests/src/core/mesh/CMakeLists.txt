add_executable(triMeshGeo_gtest triMeshGeo_gtest.cpp)
target_link_libraries(triMeshGeo_gtest PRIVATE GTest::gtest_main mesh)
set_property(TARGET triMeshGeo_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(triMeshGeo_gtest)

add_executable(meshData_gtest meshData_gtest.cpp)
target_link_libraries(meshData_gtest PRIVATE GTest::gtest_main mesh)
set_property(TARGET meshData_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(meshData_gtest)

add_executable(tetMeshGeo_gtest tetMeshGeo_gtest.cpp)
target_link_libraries(tetMeshGeo_gtest PRIVATE GTest::gtest_main mesh volumetricMesh)
set_property(TARGET tetMeshGeo_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(tetMeshGeo_gtest)

add_executable(cubicMeshGeo_gtest cubicMeshGeo_gtest.cpp)
target_link_libraries(cubicMeshGeo_gtest PRIVATE GTest::gtest_main mesh volumetricMesh)
set_property(TARGET cubicMeshGeo_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(cubicMeshGeo_gtest)

add_executable(vegFile_gtest vegFile_gtest.cpp)
target_link_libraries(vegFile_gtest PRIVATE GTest::gtest_main mesh volumetricMesh)
set_property(TARGET vegFile_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(vegFile_gtest)
