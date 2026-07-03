# Also requires one of: libbfd (gnu binutils), libdwarf, libdw (elfutils)
pgo_add_third_party(backward
  FETCHCONTENT_ARGS
    GIT_REPOSITORY https://github.com/bombela/backward-cpp
    GIT_TAG master  # or a version tag, such as v1.6
    SYSTEM          # optional, the Backward include directory will be treated as system directory
)
