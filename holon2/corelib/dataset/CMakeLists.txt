set(sources)
set(test_sources
  dataset_test.cpp
  )

holon_add_corelib_module(
  dataset
  SOURCES ${sources}
  )
holon_add_module_test(
  dataset
  SOURCES ${test_sources}
  )
