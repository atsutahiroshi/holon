set(HOLON_CORELIB_SOURCE_FILES "")
set(HOLON_CORELIB_TEST_SOURCES "")

add_subdirectory(humanoid)

add_library(holon SHARED ${HOLON_CORELIB_SOURCE_FILES})
target_include_directories(holon PUBLIC ${HOLON_INCLUDE_DIR})
target_compile_options(holon
  PRIVATE
  $<$<CXX_COMPILER_ID:Clang>:-Wall -Weverything>
  $<$<CXX_COMPILER_ID:GNU>:-pedantic -Wall -Wextra -Wshadow -Wnon-virtual-dtor>
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
  )
target_link_libraries(holon PUBLIC roki)

set(HOLON_TEST_SOURCES
  ${HOLON_CORELIB_TEST_SOURCES}
  PARENT_SCOPE
  )
