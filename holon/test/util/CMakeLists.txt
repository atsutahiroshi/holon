set(HOLON_TEST_UTIL_SOURCES "")

add_subdirectory(catch)
add_subdirectory(fuzzer)

set(HOLON_TEST_SOURCES
  ${HOLON_TEST_SOURCES}
  ${HOLON_TEST_UTIL_SOURCES}
  PARENT_SCOPE
  )
