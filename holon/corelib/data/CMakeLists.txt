set(sources)
set(test_sources
  raw_data_test.cpp
  data_set_base_test.cpp
  )

holon_add_corelib_module(
  data
  SOURCES ${sources}
  )
holon_add_module_test(
  data
  SOURCES ${test_sources}
  )
