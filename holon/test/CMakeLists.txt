set(HOLON_TEST_SOURCES
  ${HOLON_TEST_SOURCES}
  ${CMAKE_CURRENT_SOURCE_DIR}/test_main.cpp
  )

add_executable(holon_test ${HOLON_TEST_SOURCES})
target_link_libraries(holon_test PUBLIC holon Catch)
add_test(
  NAME holon_test
  COMMAND $<TARGET_FILE:holon_test> --use-colour yes
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )
