set(PERLINNOISE_HEADERS
  perlinNoise.h
  perlinWind.h
)

set(PERLINNOISE_SOURCES
  perlinNoise.cpp
  perlinWind.cpp
)

set(PERLINNOISE_DEPS eigenSupport)

pgo_require_targets(perlinNoise TARGETS ${PERLINNOISE_DEPS})

add_libpgo_lib(perlinNoise "${PERLINNOISE_SOURCES}" "${PERLINNOISE_HEADERS}")

target_link_libraries(perlinNoise PUBLIC ${PERLINNOISE_DEPS})
