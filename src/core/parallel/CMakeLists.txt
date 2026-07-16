set(PARALLEL_HEADERS
  arenaThreadingExecutor.h
  parallelControl.h
  parallel.h
  threadingPolicy.h
)

set(PARALLEL_SOURCES
  arenaThreadingExecutor.cpp
  parallelControl.cpp
  threadingPolicy.cpp
)

add_libpgo_lib(parallel "${PARALLEL_SOURCES}" "${PARALLEL_HEADERS}")

target_include_directories(parallel PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")

target_link_libraries(parallel PUBLIC TBB::tbb)

if(TARGET MKL::MKL)
  target_link_libraries(parallel PRIVATE MKL::MKL)
endif()

if(APPLE)
  target_link_libraries(parallel PRIVATE PGO::AccelerateBLAS)
endif()
