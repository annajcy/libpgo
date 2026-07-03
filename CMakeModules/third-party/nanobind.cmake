pgo_add_third_party(nanobind
  TARGETS nanobind
  STATUS "Downloading nanobind..."
  DONE_MESSAGE "nanobind ready."
  FETCHCONTENT_ARGS
    GIT_REPOSITORY https://github.com/wjakob/nanobind.git
    GIT_TAG v2.12.0
    GIT_SUBMODULES ext/robin_map
    GIT_SHALLOW TRUE
)
