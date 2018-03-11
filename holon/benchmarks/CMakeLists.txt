set(benchmark_sources
  com_ctrl_benchmark.cpp
  vec3d_computation_benchmark.cpp
  )

cmake_policy(PUSH)
if(CMAKE_VERSION VERSION_GREATER 2.8.12)
  cmake_policy(SET CMP0024 OLD)
endif()
find_package(hayai REQUIRED)
cmake_policy(POP)

function(add_benchmark source)
  if(NOT IS_ABSOLUTE ${source})
    set(source ${CMAKE_CURRENT_SOURCE_DIR}/${source})
  endif()
  get_filename_component(target ${source} NAME_WE)
  add_executable(${target} ${source})
  target_include_directories(${target} PRIVATE ${HAYAI_INCLUDE_DIRS})
  target_compile_options(${target}
    PRIVATE
    $<$<CXX_COMPILER_ID:Clang>:-Wall -Weverything $<$<CONFIG:Debug>:-g3 -O0> $<$<CONFIG:Release>:-O3>>
    $<$<CXX_COMPILER_ID:GNU>:-pedantic -Wall -Wextra -Wshadow -Wnon-virtual-dtor $<$<CONFIG:Debug>:-g3 -O0> $<$<CONFIG:Release>:-O3>>
    $<$<CXX_COMPILER_ID:MSVC>:/W4 $<$<CONFIG:Debug>:/Od> $<$<CONFIG:Release>:/O2>>
    )
  target_link_libraries(${target} PUBLIC holon hayai_main)
endfunction()

foreach(src ${benchmark_sources})
  add_benchmark(${src})
endforeach()
