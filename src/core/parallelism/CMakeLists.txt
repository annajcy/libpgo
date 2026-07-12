set(PARALLELISM_HEADERS
  parallelExecutor.h
  parallelOptions.h
  parallelFor.h
  parallelRuntime.h
)

set(PARALLELISM_SOURCES
  parallelExecutor.cpp
  parallelFor.cpp
  parallelRuntime.cpp
)

add_libpgo_lib(parallelism "${PARALLELISM_SOURCES}" "${PARALLELISM_HEADERS}")

target_include_directories(parallelism PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")

if(TARGET TBB::tbb)
  target_link_libraries(parallelism PRIVATE TBB::tbb)
  target_compile_definitions(parallelism PRIVATE PGO_PARALLELISM_HAS_TBB)
endif()

if(TARGET Eigen3::Eigen)
  target_link_libraries(parallelism PRIVATE Eigen3::Eigen)
  target_compile_definitions(parallelism PRIVATE PGO_PARALLELISM_HAS_EIGEN)
endif()

if(TARGET MKL::MKL)
  target_link_libraries(parallelism PRIVATE MKL::MKL)
  target_compile_definitions(parallelism PRIVATE PGO_PARALLELISM_HAS_MKL)
  if(PGO_MKL_THREADING STREQUAL "tbb_thread")
    target_compile_definitions(parallelism PRIVATE PGO_PARALLELISM_MKL_TBB_THREADING)
  endif()
endif()

if(APPLE)
  target_link_libraries(parallelism PRIVATE "-framework Accelerate")
  target_compile_definitions(parallelism PRIVATE PGO_PARALLELISM_HAS_ACCELERATE_THREADING)
endif()
