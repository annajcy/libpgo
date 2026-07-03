set(CONFIGFILEJSON_DEPS pgoLogging nlohmann_json::nlohmann_json)

pgo_require_targets(configFileJSON TARGETS ${CONFIGFILEJSON_DEPS})

add_libpgo_lib(configFileJSON configFileJSON.cpp configFileJSON.h)

target_link_libraries(configFileJSON PUBLIC ${CONFIGFILEJSON_DEPS})
