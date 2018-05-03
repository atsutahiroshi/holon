set(sources
  com_zmp_model.cpp
  const_defs.cpp
  )
set(test_sources
  com_zmp_model_test.cpp
  )
holon_add_corelib_module(
  humanoid
  SOURCES ${sources}
  )
holon_add_module_test(
  humanoid
  SOURCES ${test_sources}
  )
