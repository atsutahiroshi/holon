set(HOLON_BENCHMARK_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/com_ctrl_benchmark.cpp
  )

cmake_policy(PUSH)
if(CMAKE_VERSION VERSION_GREATER 2.8.12)
  cmake_policy(SET CMP0024 OLD)
endif()
find_package(hayai REQUIRED)
cmake_policy(POP)

function(add_benchmark src)
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
  target_include_directories(${target} PRIVATE ${HAYAI_INCLUDE_DIRS})
  target_link_libraries(${target} PUBLIC holon hayai_main)
endfunction()

foreach(src ${HOLON_BENCHMARK_SOURCES})
  add_benchmark(${src})
endforeach()
