set(HOLON_HUMANOID_EXAMPLE_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/com_regulation_open_loop.cpp
  )

foreach(src ${HOLON_HUMANOID_EXAMPLE_SOURCES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
  target_compile_options(${target}
    PRIVATE
    $<$<CXX_COMPILER_ID:Clang>:-Wall -Weverything $<$<CONFIG:Debug>:-g3 -O0> $<$<CONFIG:Release>:-O3>>
    $<$<CXX_COMPILER_ID:GNU>:-pedantic -Wall -Wextra -Wshadow -Wnon-virtual-dtor $<$<CONFIG:Debug>:-g3 -O0> $<$<CONFIG:Release>:-O3>>
    $<$<CXX_COMPILER_ID:MSVC>:/W4 $<$<CONFIG:Debug>:/Od> $<$<CONFIG:Release>:/O2>>
    )
  target_link_libraries(${target} PUBLIC holon)
endforeach()
