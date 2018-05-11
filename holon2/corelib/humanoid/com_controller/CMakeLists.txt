set(sources
  contact_force_generator.cpp
  zmp_manipulator.cpp
  )
set(test_sources
  contact_force_generator_test.cpp
  zmp_manipulator_test.cpp
  zmp_manipulator_x_test.cpp
  zmp_manipulator_y_test.cpp
  )

holon_add_corelib_module_source(humanoid
  PREPEND
  SOURCES ${sources}
  )
holon_add_module_test_source(humanoid
  PREPEND
  SOURCES ${test_sources}
  )
