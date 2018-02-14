set(HOLON_CORELIB_SOURCE_FILES "")
set(HOLON_CORELIB_TEST_SOURCES "")

add_subdirectory(math)
add_subdirectory(humanoid)

add_library(holon SHARED ${HOLON_CORELIB_SOURCE_FILES})
target_include_directories(holon PUBLIC ${HOLON_INCLUDE_DIR})
target_compile_options(holon
  PRIVATE
  $<$<CXX_COMPILER_ID:Clang>:-Wall -Weverything $<$<CONFIG:Debug>:-g3 -O0> $<$<CONFIG:Release>:-O3>>
  $<$<CXX_COMPILER_ID:GNU>:-pedantic -Wall -Wextra -Wshadow -Wnon-virtual-dtor $<$<CONFIG:Debug>:-g3 -O0> $<$<CONFIG:Release>:-O3>>
  $<$<CXX_COMPILER_ID:MSVC>:/W4 $<$<CONFIG:Debug>:/Od> $<$<CONFIG:Release>:/O2>>
  )
target_link_libraries(holon PUBLIC roki)

set(HOLON_TEST_SOURCES
  ${HOLON_CORELIB_TEST_SOURCES}
  PARENT_SCOPE
  )
