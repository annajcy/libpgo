if(TARGET geogram::geogram)
  return()
endif()

pgo_dep_option(GEOGRAM_SUB_BUILD BOOL ON "Building as subproject")
pgo_dep_option(GEOGRAM_BUILD_SHARED BOOL OFF "Build geogram shared library")
pgo_dep_option(GEOGRAM_BUILD_STATIC BOOL ON "Build geogram static library")
pgo_dep_option(GEOGRAM_LIB_ONLY BOOL ON "Build geogram lib only")
pgo_dep_option(GEOGRAM_WITH_GRAPHICS BOOL OFF "Disable graphics")
pgo_dep_option(GEOGRAM_WITH_HLBFGS BOOL ON "Non-linear solver (Yang Liu's HLBFGS)")
pgo_dep_option(GEOGRAM_WITH_LUA BOOL OFF "Disable LUA")
pgo_dep_option(GEOGRAM_WITH_EXPLORAGRAM BOOL OFF "Disable exploragram")
pgo_dep_option(GEOGRAM_WITH_LEGACY_NUMERICS BOOL OFF "Disable legacy numerics")
pgo_dep_option(GEOGRAM_WITH_TRIANGLE BOOL OFF "Disable triangle")

function(_libpgo_remove_geogram_linux_thread_flags target_file)
  pgo_replace_in_file(
    "${target_file}"
    [=[if (GCC_VERSION VERSION_GREATER 4.0)
    add_flags(CMAKE_CXX_FLAGS -fopenmp)
    add_flags(CMAKE_C_FLAGS -fopenmp)
endif()]=]
    ""
  )
endfunction()

function(_pgo_setup_geogram)
  set(MODIFIED_FILE "${CMAKE_SOURCE_DIR}/CMakeModules/patches/geogram.cmake")
  set(TARGET_FILE "${geogram_SOURCE_DIR}/CMakeLists.txt")

  pgo_copy_file("${MODIFIED_FILE}" "${TARGET_FILE}")
  _libpgo_remove_geogram_linux_thread_flags("${geogram_SOURCE_DIR}/cmake/platforms/Linux-gcc.cmake")

  set(POISSON_RECON_DIR "${geogram_SOURCE_DIR}/src/lib/geogram/third_party/PoissonRecon")

  pgo_replace_in_file(
    "${POISSON_RECON_DIR}/SparseMatrix.inl"
    [=[void SparseMatrix<T>::SetZero()
{
        Resize(this->m_N, this->m_M);
}]=]
    [=[void SparseMatrix<T>::SetZero()
{
        for( int i=0 ; i<rows ; i++ ) for( int ii=0 ; ii<rowSizes[i] ; ii++ ) m_ppElements[i][ii].Value = T(0);
}]=]
  )

  pgo_replace_in_file(
    "${POISSON_RECON_DIR}/SparseMatrix.inl"
    [=[for( int i=0 ; i<rows ; i++ ) for( int ii=0 ; ii<rowSizes[i] ; i++ ) m_ppElements[i][ii].Value *= V;]=]
    [=[for( int i=0 ; i<rows ; i++ ) for( int ii=0 ; ii<rowSizes[i] ; ii++ ) m_ppElements[i][ii].Value *= V;]=]
  )

  pgo_replace_in_file(
    "${POISSON_RECON_DIR}/PlyVertexMini.h"
    [=[        PlyValueVertex operator - ( PlyValueVertex p ) const { return PlyValueVertex( point-p.value , value-p.value ); }]=]
    [=[        PlyValueVertex operator - ( PlyValueVertex p ) const { return PlyValueVertex( point-p.point , value-p.value ); }]=]
  )

  pgo_replace_in_file(
    "${POISSON_RECON_DIR}/PlyVertexMini.h"
    [=[        PlyOrientedVertex operator - ( PlyOrientedVertex p ) const { return PlyOrientedVertex( point-p.value , normal-p.normal ); }]=]
    [=[        PlyOrientedVertex operator - ( PlyOrientedVertex p ) const { return PlyOrientedVertex( point-p.point , normal-p.normal ); }]=]
  )

  pgo_replace_in_file(
    "${POISSON_RECON_DIR}/PlyVertexMini.h"
    [=[                _PlyColorVertex operator - ( _PlyColorVertex p ) const { return _PlyColorVertex( point-p.value , color-p.color ); }]=]
    [=[                _PlyColorVertex operator - ( _PlyColorVertex p ) const { return _PlyColorVertex( point-p.point , color-p.color ); }]=]
  )

  pgo_replace_in_file(
    "${POISSON_RECON_DIR}/PlyVertexMini.h"
    [=[                _PlyColorAndValueVertex operator - ( _PlyColorAndValueVertex p ) const { return _PlyColorAndValueVertex( point-p.value , color-p.color , value+p.value ); }]=]
    [=[                _PlyColorAndValueVertex operator - ( _PlyColorAndValueVertex p ) const { return _PlyColorAndValueVertex( point-p.point , color-p.color , value-p.value ); }]=]
  )

  pgo_add_populated_subdirectory(geogram)
  set_target_properties(geogram PROPERTIES CXX_STANDARD 14)
endfunction()

pgo_add_third_party(geogram
  TARGETS geogram::geogram
  STATUS "Loading geogram..."
  FETCH_MODE POPULATE
  POPULATE_REASON "geogram source tree is patched before add_subdirectory"
  POST_FETCH _pgo_setup_geogram
  FETCHCONTENT_ARGS
    URL https://github.com/BrunoLevy/geogram/releases/download/v1.9.0/geogram_1.9.0.zip
    EXCLUDE_FROM_ALL
    DOWNLOAD_EXTRACT_TIMESTAMP ON
)
