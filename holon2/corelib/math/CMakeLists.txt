set(sources
  vec.cpp
  )
set(test_sources
  mat_test.cpp
  misc_test.cpp
  ode_euler_test.cpp
  ode_runge_kutta4_test.cpp
  )

holon_add_corelib_module(
  math
  SOURCES ${sources}
  )
holon_add_module_test(
  math
  SOURCES ${test_sources}
  )
