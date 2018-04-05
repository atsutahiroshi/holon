set(sources
  biped_model.cpp
  com_ctrl.cpp
  com_zmp_model.cpp
  )
set(test_sources
  biped_model_test.cpp
  com_ctrl_test.cpp
  com_zmp_model_test.cpp
  )

holon_add_corelib_module(
  humanoid
  SOURCES ${sources}
  )
holon_add_module_test(
  humanoid
  SOURCES ${test_sources}
  )

add_subdirectory(com_ctrl)
add_subdirectory(com_zmp_model)
