set(GEOGRAM_INTERFACE_HEADERS geogramInterface.h)
set(GEOGRAM_INTERFACE_SOURCES geogramInterface.cpp)
set(GEOGRAM_INTERFACE_DEPS pgoLogging eigenSupport mesh geogram)

pgo_require_targets(geogramInterface TARGETS ${GEOGRAM_INTERFACE_DEPS})

add_libpgo_lib(geogramInterface "${GEOGRAM_INTERFACE_SOURCES}" "${GEOGRAM_INTERFACE_HEADERS}")

target_link_libraries(geogramInterface PUBLIC ${GEOGRAM_INTERFACE_DEPS})
