set(TETGEN_INTERFACE_HEADERS tetgenInterface.h)
set(TETGEN_INTERFACE_SOURCES tetgenInterface.cpp)
set(TETGEN_INTERFACE_DEPS pgoLogging eigenSupport mesh tetgen)

pgo_require_targets(tetgenInterface TARGETS ${TETGEN_INTERFACE_DEPS})

add_libpgo_lib(tetgenInterface "${TETGEN_INTERFACE_SOURCES}" "${TETGEN_INTERFACE_HEADERS}")

target_link_libraries(tetgenInterface PUBLIC ${TETGEN_INTERFACE_DEPS})
