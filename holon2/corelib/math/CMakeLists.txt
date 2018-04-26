set(sources
  dummy.cpp
  )
set(test_sources
  mat_test.cpp
  )

holon_add_corelib_module(
  math
  SOURCES ${sources}
  )
holon_add_module_test(
  math
  SOURCES ${test_sources}
  )
