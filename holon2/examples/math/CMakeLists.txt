set(example_sources
  ode_euler_example.cpp
  ode_runge_kutta4_example.cpp
  )

foreach(src ${example_sources})
  holon_add_example(${src})
endforeach()

add_subdirectory(tests)
add_subdirectory(tools)
