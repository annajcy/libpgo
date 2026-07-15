set(PARALLEL_HEADERS
  accelerateThreading.h
  parallelControl.h
  parallel.h
  parallelFor.h
  parallelReduce.h
  parallelSort.h
)

set(PARALLEL_SOURCES
  accelerateThreading.cpp
  parallelControl.cpp
)

add_libpgo_lib(parallel "${PARALLEL_SOURCES}" "${PARALLEL_HEADERS}")

target_include_directories(parallel PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")

target_link_libraries(parallel PUBLIC TBB::tbb)

if(APPLE)
  target_link_libraries(parallel PRIVATE PGO::AccelerateBLAS)
endif()
