set(EIGENSUPPORT_HEADERS
  EigenSupport.h
  EigenDef.h
)

set(EIGENSUPPORT_SOURCES
  EigenSupport.cpp
)

if(TARGET MKL::MKL)
  list(APPEND EIGENSUPPORT_HEADERS EigenMKLPardisoSupport.h)
  list(APPEND EIGENSUPPORT_SOURCES EigenMKLPardisoSupport.cpp)
endif()

set(_libpgo_enable_arpack_support OFF)
if(PGO_FEATURE_ARPACK AND TARGET MKL::MKL AND TARGET ARPACK::ARPACK)
  set(_libpgo_enable_arpack_support ON)
  message(STATUS "has arpack")
  list(APPEND EIGENSUPPORT_HEADERS EigenARPACKSupport.h)
  list(APPEND EIGENSUPPORT_SOURCES EigenARPACKSupport.cpp)
elseif(PGO_FEATURE_ARPACK AND NOT TARGET MKL::MKL)
  message(WARNING "PGO_FEATURE_ARPACK=ON but MKL is disabled; EigenARPACKSupport is skipped.")
elseif(PGO_FEATURE_ARPACK AND NOT TARGET ARPACK::ARPACK)
  message(WARNING "PGO_FEATURE_ARPACK=ON but ARPACK::ARPACK is unavailable; EigenARPACKSupport is skipped.")
else()
  message(STATUS "has no arpack")
endif()

set(EIGENSUPPORT_DEPS Eigen3::Eigen TBB::tbb nlohmann_json::nlohmann_json)

foreach(tgt ${EIGENSUPPORT_DEPS})
  if(NOT TARGET ${tgt})
    message(STATUS "eigenSupport is not included. Missing: ${tgt}")
    return()
  endif()
endforeach()

add_libpgo_lib(eigenSupport "${EIGENSUPPORT_SOURCES}" "${EIGENSUPPORT_HEADERS}")

target_link_libraries(eigenSupport PUBLIC ${EIGENSUPPORT_DEPS})

if(_libpgo_enable_arpack_support)
  target_link_libraries(eigenSupport PUBLIC ARPACK::ARPACK)
  target_compile_definitions(eigenSupport PUBLIC PGO_HAS_ARPACK)
endif()

unset(_libpgo_enable_arpack_support)
