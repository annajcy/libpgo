set(INTERPOLATIONCOORDINATES_HEADERS interpolationCoordinatesBase.h meanValueCoordinates.h GreenCoordinates.h barycentricCoordinates.h)
set(INTERPOLATIONCOORDINATES_SOURCES meanValueCoordinates.cpp GreenCoordinates.cpp barycentricCoordinates.cpp)
set(INTERPOLATIONCOORDINATES_DEPS volumetricMesh)

pgo_require_targets(interpolationCoordinates TARGETS ${INTERPOLATIONCOORDINATES_DEPS} parallel TBB::tbb)

add_libpgo_lib(interpolationCoordinates "${INTERPOLATIONCOORDINATES_SOURCES}" "${INTERPOLATIONCOORDINATES_HEADERS}")

target_link_libraries(interpolationCoordinates PUBLIC ${INTERPOLATIONCOORDINATES_DEPS})
target_link_libraries(interpolationCoordinates PRIVATE parallel TBB::tbb)
