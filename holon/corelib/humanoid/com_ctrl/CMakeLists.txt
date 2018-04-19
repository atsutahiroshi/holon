set(sources
  com_guided_ctrl_formula.cpp
  )
set(test_sources
  com_guided_ctrl_formula_test.cpp
  com_guided_ctrl_formula_x_test.cpp
  com_guided_ctrl_formula_y_test.cpp
  com_guided_ctrl_formula_z_test.cpp
  com_guided_ctrl_formula_phase_test.cpp
  )

holon_add_corelib_module_source(humanoid
  PREPEND
  SOURCES ${sources}
  )
holon_add_module_test_source(humanoid
  PREPEND
  SOURCES ${test_sources}
  )
