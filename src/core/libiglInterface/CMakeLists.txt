set(LIBIGL_INTERFACE_HEADERS libiglInterface.h)
set(LIBIGL_INTERFACE_SOURCES libiglInterface.cpp)
set(LIBIGL_INTERFACE_DEPS pgoLogging eigenSupport mesh igl::core igl_copyleft::cgal)

pgo_require_targets(libigl TARGETS ${LIBIGL_INTERFACE_DEPS})

add_libpgo_lib(libiglInterface "${LIBIGL_INTERFACE_SOURCES}" "${LIBIGL_INTERFACE_HEADERS}")

target_link_libraries(libiglInterface PUBLIC ${LIBIGL_INTERFACE_DEPS})
