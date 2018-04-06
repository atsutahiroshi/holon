set(example_sources
  biped_model_walk_example.cpp
  com_longitudinal_moving_example.cpp
  com_oscillation_example.cpp
  com_regulation_example.cpp
  com_regulation_vertical_example.cpp
  com_regulation_with_disturbance_example.cpp
  com_sideward_moving_example.cpp
  )

foreach(src ${example_sources})
  holon_add_example(${src})
endforeach()

add_subdirectory(tests)
add_subdirectory(tools)
