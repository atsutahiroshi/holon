set(sources)
set(test_sources
  point_mass_model_test.cpp
  )

holon_add_corelib_module(
  control
  SOURCES ${sources}
  )
holon_add_module_test(
  control
  SOURCES ${test_sources}
  )

add_subdirectory(point_mass_model)
