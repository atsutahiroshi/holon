set(HOLON_EXAMPLE_HUMANOID_TEST_SCRIPTS
  ${CMAKE_CURRENT_SOURCE_DIR}/com_longitudinal_moving_example_test.py
  ${CMAKE_CURRENT_SOURCE_DIR}/com_oscillation_example_test.py
  ${CMAKE_CURRENT_SOURCE_DIR}/com_regulation_example_test.py
  ${CMAKE_CURRENT_SOURCE_DIR}/com_regulation_vertical_example_test.py
  ${CMAKE_CURRENT_SOURCE_DIR}/com_regulation_with_disturbance_example_test.py
  )

find_package(PythonInterp 3.4)
find_package(NumPy)
foreach(src_script ${HOLON_EXAMPLE_HUMANOID_TEST_SCRIPTS})
  get_filename_component(filename ${src_script} NAME)
  get_filename_component(testname ${src_script} NAME_WE)
  set(output ${CMAKE_CURRENT_BINARY_DIR}/${filename})
  add_custom_command(
    OUTPUT ${output}
    COMMAND ${CMAKE_COMMAND} -E copy ${src_script} ${output}
    DEPENDS ${src_script}
    )
  add_custom_target(
    ${filename} ALL
    DEPENDS ${output}
    )
  if(${PYTHON_NUMPY_FOUND})
    add_test(
      NAME ${testname}
      COMMAND ${PYTHON_EXECUTABLE} ${filename}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      )
  endif()
endforeach()

# copy data files
file(GLOB HOLON_EXAMPLES_HUMANOID_TEST_DATA_FILES data/*.dat)
foreach(path ${HOLON_EXAMPLES_HUMANOID_TEST_DATA_FILES})
  get_filename_component(file ${path} NAME)
  configure_file(data/${file} data/${file} COPYONLY)
endforeach()
