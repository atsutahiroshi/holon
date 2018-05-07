set(sources
  com_zmp_model_builder.cpp
  com_zmp_model_formula.cpp
  com_zmp_model_simulator.cpp
  )
set(test_sources
  com_zmp_model_builder_test.cpp
  com_zmp_model_formula_test.cpp
  com_zmp_model_simulator_test.cpp
  )

holon_add_corelib_module_source(humanoid
  PREPEND
  SOURCES ${sources}
  )
holon_add_module_test_source(humanoid
  PREPEND
  SOURCES ${test_sources}
  )
