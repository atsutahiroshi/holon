set(sources
  com_ctrl_x.cpp
  com_ctrl_y.cpp
  com_ctrl_z.cpp
  phase_y.cpp
  )
set(test_sources
  com_ctrl_x_test.cpp
  com_ctrl_y_test.cpp
  com_ctrl_z_test.cpp
  phase_y_test.cpp
  )

holon_add_corelib_module_source(humanoid
  PREPEND
  SOURCES ${sources}
  )
holon_add_module_test_source(humanoid
  PREPEND
  SOURCES ${test_sources}
  )
