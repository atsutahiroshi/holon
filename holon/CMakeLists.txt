set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

include(Functions)
holon_set_var(HOLON_INCLUDE_DIR ${PROJECT_SOURCE_DIR})

add_subdirectory(corelib)
add_subdirectory(modules)
add_subdirectory(test)
add_subdirectory(examples)
add_subdirectory(benchmarks)
