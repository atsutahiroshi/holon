set(test_scripts
  com_longitudinal_moving_example_test.py
  com_oscillation_example_test.py
  com_regulation_example_test.py
  com_regulation_vertical_example_test.py
  com_regulation_with_disturbance_example_test.py
  )

find_package(PythonInterp 3.4)
find_package(NumPy)
foreach(script ${test_scripts})
  holon_add_test_script(${script})
endforeach()

# copy data files
file(GLOB test_data_files data/*.dat)
foreach(path ${test_data_files})
  get_filename_component(file ${path} NAME)
  configure_file(data/${file} data/${file} COPYONLY)
endforeach()
