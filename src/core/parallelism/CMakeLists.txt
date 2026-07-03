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

if(TARGET Eigen3::Eigen)
  target_link_libraries(parallelism PRIVATE Eigen3::Eigen)
  target_compile_definitions(parallelism PRIVATE PGO_PARALLELISM_HAS_EIGEN)
endif()

if(PGO_OPENMP_FOUND)
  target_compile_definitions(parallelism PRIVATE PGO_PARALLELISM_HAS_OPENMP)
  if(TARGET OpenMP::OpenMP_CXX)
    target_link_libraries(parallelism PRIVATE OpenMP::OpenMP_CXX)
  endif()
endif()

if(TARGET MKL::MKL)
  target_link_libraries(parallelism PRIVATE MKL::MKL)
  target_compile_definitions(parallelism PRIVATE PGO_PARALLELISM_HAS_MKL)
  if(PGO_MKL_THREADING STREQUAL "tbb_thread")
    target_compile_definitions(parallelism PRIVATE PGO_PARALLELISM_MKL_TBB_THREADING)
  endif()
elseif(BLA_VENDOR STREQUAL "OpenBLAS" AND TARGET BLAS::BLAS)
  target_link_libraries(parallelism PRIVATE BLAS::BLAS)
  target_compile_definitions(parallelism PRIVATE PGO_PARALLELISM_HAS_OPENBLAS)
endif()
