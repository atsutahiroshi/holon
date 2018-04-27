cmake_minimum_required(VERSION 3.2)

project(holon CXX C)
option(TESTING_HOLON2 "Testing holon2" ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()
add_subdirectory(third_party)
if(TESTING_HOLON2)
  add_subdirectory(holon2)
else()
  add_subdirectory(holon)
endif()
