if(NOT PGO_ENABLE_TESTS)
	return()
endif()

enable_testing()

find_package(GTest REQUIRED)

add_subdirectory(core)
add_subdirectory(api)
add_subdirectory(tools)
