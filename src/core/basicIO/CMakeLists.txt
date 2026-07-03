pgo_require_targets(basicIO TARGETS pgoLogging)

add_libpgo_lib(basicIO basicIO.cpp basicIO.h)
target_link_libraries(basicIO PUBLIC pgoLogging)
