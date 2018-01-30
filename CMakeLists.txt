cmake_minimum_required(VERSION 3.2)

project(holon CXX C)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(HOLON_SOURCE_DIR ${CMAKE_SOURCE_DIR}/holon/src)
set(HOLON_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/holon/include)
set(HOLON_TEST_DIR ${CMAKE_SOURCE_DIR}/holon/test)

set(HOLON_SOURCE_FILES
  ${HOLON_SOURCE_DIR}/com_zmp_model.cpp
  )
set(HOLON_HEADER_FILES
  ${HOLON_INCLUDE_DIR}/com_zmp_model.hpp
  )
set(HOLON_TEST_SOURCES
  ${HOLON_TEST_DIR}/test_com_zmp_model.cpp
  ${HOLON_TEST_DIR}/test_main.cpp
  )

add_subdirectory(third_party)

add_library(holon ${HOLON_SOURCE_FILES})
target_include_directories(holon PUBLIC ${HOLON_INCLUDE_DIR})
target_compile_options(holon
  PRIVATE
  $<$<CXX_COMPILER_ID:Clang>:-Wall -Weverything>
  $<$<CXX_COMPILER_ID:GNU>:-pedantic -Wall -Wextra -Wshadow -Wnon-virtual-dtor>
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
  )

enable_testing()
add_executable(test_main ${HOLON_TEST_SOURCES})
target_link_libraries(test_main PUBLIC holon Catch)
add_test(NAME holon_test COMMAND test_main --use-colour yes)
