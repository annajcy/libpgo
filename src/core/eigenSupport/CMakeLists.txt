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

if(PGO_HAS_ORIG_PARDISO)
  list(APPEND EIGENSUPPORT_HEADERS EigenOrigPardisoSupport.h)
  list(APPEND EIGENSUPPORT_SOURCES EigenOrigPardisoSupport.cpp)
endif()

# if(PGO_HAS_ARPACK)
#   message(STATUS "has arpack")
#   list(APPEND EIGENSUPPORT_HEADERS EigenARPACKSupport.h)
#   list(APPEND EIGENSUPPORT_SOURCES EigenARPACKSupport.cpp)
# else()
#   message(STATUS "has no arpack")
# endif()

set(EIGENSUPPORT_DEPS Eigen3::Eigen TBB::tbb nlohmann_json::nlohmann_json)

pgo_require_targets(eigenSupport TARGETS ${EIGENSUPPORT_DEPS} parallelism)

add_libpgo_lib(eigenSupport "${EIGENSUPPORT_SOURCES}" "${EIGENSUPPORT_HEADERS}")

target_link_libraries(eigenSupport PUBLIC ${EIGENSUPPORT_DEPS})
target_link_libraries(eigenSupport PRIVATE parallelism)

# if(PGO_HAS_ARPACK)
#   target_link_libraries(eigenSupport PUBLIC arpack ${GFORTRAN_LIBRARY} ${QUADMATH_LIBRARY})
#   target_compile_definitions(eigenSupport PUBLIC PGO_HAS_ARPACK)
# endif()

if(PGO_HAS_ORIG_PARDISO)
  target_link_libraries(eigenSupport PUBLIC Pardiso::Pardiso)
endif()
