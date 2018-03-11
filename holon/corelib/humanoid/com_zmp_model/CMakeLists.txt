set(sources
  com_zmp_model_data.cpp
  com_zmp_model_formula.cpp
  com_zmp_model_system.cpp
  )
set(test_sources
  com_zmp_model_data_test.cpp
  com_zmp_model_formula_test.cpp
  com_zmp_model_system_test.cpp
  )

holon_add_corelib_module_source(humanoid
  PREPEND
  SOURCES ${sources}
  )
holon_add_module_test_source(humanoid
  PREPEND
  SOURCES ${test_sources}
  )
