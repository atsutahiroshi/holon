set(sources
  vec3d.cpp
  )
set(test_sources
  vec3d_test.cpp
  )

holon_add_corelib_module_source(math
  PREPEND
  SOURCES ${sources}
  )
holon_add_module_test_source(math
  PREPEND
  SOURCES ${test_sources}
  )
