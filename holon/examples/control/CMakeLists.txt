set(example_sources
  pd_ctrl_example.cpp
  )

foreach(src ${example_sources})
  holon_add_example(${src})
endforeach()

add_subdirectory(tests)
add_subdirectory(tools)
