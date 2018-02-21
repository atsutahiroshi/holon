set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")
set(HOLON_INCLUDE_DIR ${PROJECT_SOURCE_DIR})

set(HOLON_TEST_SOURCES "")

add_subdirectory(corelib)
add_subdirectory(modules)
add_subdirectory(test)
add_subdirectory(examples)
add_subdirectory(benchmarks)
