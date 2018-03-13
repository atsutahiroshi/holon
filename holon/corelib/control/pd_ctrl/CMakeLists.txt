set(sources)
set(test_sources
  pd_ctrl_formula_test.cpp
  )

holon_add_corelib_module_source(control
  PREPEND
  SOURCES ${sources}
  )
holon_add_module_test_source(control
  PREPEND
  SOURCES ${test_sources}
  )
