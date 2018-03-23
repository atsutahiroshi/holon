set(sources)
set(test_sources
  model_base_test.cpp
  pd_ctrl_test.cpp
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

add_subdirectory(pd_ctrl)
add_subdirectory(point_mass_model)
