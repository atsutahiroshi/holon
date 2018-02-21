

configure_file(
  data/com_regulation_example.dat
  data/com_regulation_example.dat
  COPYONLY
  )

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/com_regulation_example_test.py
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/com_regulation_example_test.py ${CMAKE_CURRENT_BINARY_DIR}/com_regulation_example_test.py
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/com_regulation_example_test.py
  )

add_custom_target(
  com_regulation_example_test.py ALL
  DEPENDS com_regulation_example
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/com_regulation_example_test.py
  )

find_package(PythonInterp 3.4)
find_package(NumPy)
if(${PYTHON_NUMPY_FOUND})
  add_test(
    NAME com_regulation_example_test
    COMMAND ${PYTHON_EXECUTABLE} com_regulation_example_test.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
endif()
