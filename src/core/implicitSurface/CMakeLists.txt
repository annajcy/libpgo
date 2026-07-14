set(IMPLICIT_SURFACE_HEADERS
  core/ImplicitField.h
  field/gridSpec.h
  fields/GridField.h
  fields/SphereField.h
  fields/MeshUnsignedDistanceField.h
  fields/BoxField.h
  operations/booleanOps.h
  operations/OffsetField.h
  extraction/marchingCubesExtractor.h)

set(IMPLICIT_SURFACE_SOURCES
  core/ImplicitField.cpp
  field/gridSpec.cpp
  fields/GridField.cpp
  fields/SphereField.cpp
  fields/MeshUnsignedDistanceField.cpp
  operations/booleanOps.cpp
  extraction/marchingCubesExtractor.cpp)

set(IMPLICIT_SURFACE_DEPS pgoLogging eigenSupport mesh libiglInterface parallel)

pgo_require_targets(implicitSurface TARGETS ${IMPLICIT_SURFACE_DEPS})

# The extractor TU is always compiled; it provides throwing stubs when OpenVDB
# is unavailable, so the API is present regardless of backend.
list(APPEND IMPLICIT_SURFACE_SOURCES extraction/openVDBExtractor.cpp)
list(APPEND IMPLICIT_SURFACE_HEADERS extraction/openVDBExtractor.h)

set(PGO_HAS_OPENVDB 0)
if(TARGET openvdb_static)
  list(APPEND IMPLICIT_SURFACE_DEPS openvdb_static)
  set(PGO_HAS_OPENVDB 1)
elseif(TARGET OpenVDB::openvdb)
  list(APPEND IMPLICIT_SURFACE_DEPS OpenVDB::openvdb)
  set(PGO_HAS_OPENVDB 1)
else()
  message(STATUS "implicitSurface: OpenVDB not found. Building with marching-cubes backend only.")
endif()

add_libpgo_lib(implicitSurface "${IMPLICIT_SURFACE_SOURCES}" "${IMPLICIT_SURFACE_HEADERS}")
target_link_libraries(implicitSurface PUBLIC ${IMPLICIT_SURFACE_DEPS})
if(PGO_HAS_OPENVDB)
  target_compile_definitions(implicitSurface PUBLIC PGO_HAS_OPENVDB)
endif()
