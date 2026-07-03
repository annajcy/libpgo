set(PGOLOGGING_HEADERS pgoLogging.h fmtEigen.h)
set(PGOLOGGING_DEPS Eigen3::Eigen spdlog::spdlog fmt::fmt-header-only)

pgo_require_targets(pgoLogging TARGETS ${PGOLOGGING_DEPS})

add_libpgo_lib(pgoLogging pgoLogging.cpp "${PGOLOGGING_HEADERS}")

target_link_libraries(pgoLogging PUBLIC ${PGOLOGGING_DEPS})
target_compile_definitions(pgoLogging PUBLIC FMT_UNICODE=0)
