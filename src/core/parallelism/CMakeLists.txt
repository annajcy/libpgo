set(PARALLELISM_HEADERS
  parallelOptions.h
  parallelFor.h
)

set(PARALLELISM_SOURCES
  parallelFor.cpp
)

add_libpgo_lib(parallelism "${PARALLELISM_SOURCES}" "${PARALLELISM_HEADERS}")

target_include_directories(parallelism PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")

if(TARGET TBB::tbb)
  target_link_libraries(parallelism PRIVATE TBB::tbb)
  target_compile_definitions(parallelism PRIVATE PGO_PARALLELISM_HAS_TBB)
endif()

if(OPENMP_FOUND)
  target_compile_definitions(parallelism PRIVATE PGO_PARALLELISM_HAS_OPENMP)
endif()

