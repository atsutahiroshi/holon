set(sources
  fuzzer.cpp
  )
set(test_sources
  fuzzer_test.cpp
  )

holon_add_test_source(SOURCES ${sources})
holon_add_module_test_source(
  test_util
  SOURCES ${test_sources}
  )
