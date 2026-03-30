if(NOT TARGET fileService)
  message(STATUS "core_file_service_test is not included. Missing target: fileService")
  return()
endif()

add_libpgo_tools(core_file_service_test fileService_test.cpp fileService)
target_link_libraries(core_file_service_test PRIVATE GTest::gtest_main)
set_property(TARGET core_file_service_test PROPERTY FOLDER "tests")

add_test(NAME core.file_service.path_resolution COMMAND core_file_service_test)