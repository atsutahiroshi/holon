set(sources)
set(test_sources
  random_test.cpp
  utility_test.cpp
  optional_test.cpp
  )

holon_add_corelib_module(
  common
  SOURCES ${sources}
  )
holon_add_module_test(
  common
  SOURCES ${test_sources}
  )
