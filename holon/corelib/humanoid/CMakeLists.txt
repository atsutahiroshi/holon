set(HOLON_CORELIB_HUMANOID_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/com_ctrl.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/com_zmp_model.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/point_mass_model.cpp
  )
set(HOLON_CORELIB_HUMANOID_TEST_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/com_ctrl_test.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/com_zmp_model_test.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/point_mass_model_test.cpp
  )

add_subdirectory(com_ctrl)
add_subdirectory(com_zmp_model)
add_subdirectory(point_mass_model)

set(HOLON_CORELIB_SOURCE_FILES
  ${HOLON_CORELIB_SOURCE_FILES}
  ${HOLON_CORELIB_HUMANOID_SOURCE_FILES}
  PARENT_SCOPE
  )
set(HOLON_CORELIB_TEST_SOURCES
  ${HOLON_CORELIB_TEST_SOURCES}
  ${HOLON_CORELIB_HUMANOID_TEST_SOURCES}
  PARENT_SCOPE
  )
