set(sources
  zmp_manipulator.cpp
  )
set(test_sources
  zmp_manipulator_test.cpp
  # zmp_manipulator_x_test.cpp
  )

holon_add_corelib_module_source(humanoid
  PREPEND
  SOURCES ${sources}
  )
holon_add_module_test_source(humanoid
  PREPEND
  SOURCES ${test_sources}
  )
