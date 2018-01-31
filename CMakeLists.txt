cmake_minimum_required(VERSION 3.2)

project(holon CXX C)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(HOLON_INCLUDE_DIR ${PROJECT_SOURCE_DIR})

enable_testing()
add_subdirectory(third_party)
add_subdirectory(holon)
