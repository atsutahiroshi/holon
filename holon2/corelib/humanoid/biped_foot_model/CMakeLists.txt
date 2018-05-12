set(sources
  biped_foot_model_builder.cpp
  )
set(test_sources
  biped_foot_model_builder_test.cpp
  )

holon_add_corelib_module_source(humanoid
  PREPEND
  SOURCES ${sources}
  )
holon_add_module_test_source(humanoid
  PREPEND
  SOURCES ${test_sources}
  )
