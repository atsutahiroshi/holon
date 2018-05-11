set(sources
  biped_foot_model.cpp
  com_controller.cpp
  com_zmp_model.cpp
  const_defs.cpp
  )
set(test_sources
  biped_foot_model_test.cpp
  com_controller_test.cpp
  com_zmp_model_test.cpp
  simulator_test.cpp
  )
holon_add_corelib_module(
  humanoid
  SOURCES ${sources}
  )
holon_add_module_test(
  humanoid
  SOURCES ${test_sources}
  )

add_subdirectory(biped_foot_model)
add_subdirectory(com_controller)
add_subdirectory(com_zmp_model)
