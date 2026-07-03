pgo_add_third_party(libigl
  TARGETS igl::core
  STATUS "Loading libigl..."
  FETCHCONTENT_ARGS
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG main
)
