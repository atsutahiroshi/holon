include(CMakeParseArguments)

# Create an empty variable which is cached internally.
# Usage:
#   holon_create_var(HOLON_CORELIB_SOURCE_FILES)
function(holon_create_var var_name)
  set(${var_name} ""
    CACHE INTERNAL ""
    )
endfunction(holon_create_var)


# Set values to an internally cached variable.
# Usage:
#   holon_set_var(HOLON_CORELIB_SOURCE_FILES
#     ${CMAKE_CURRENT_SOURCE_DIR}/a.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/b.cpp
#     )
function(holon_set_var var_name)
  set(${var_name} ${ARGN}
    CACHE INTERNAL ""
    )
endfunction(holon_set_var)


# Append values to an internally cached variable.
# Usage:
#   holon_list_append(HOLON_CORELIB_SOURCE_FILES
#     ${CMAKE_CURRENT_SOURCE_DIR}/c.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/d.cpp
#     )
function(holon_list_append list_name)
  holon_set_var(${list_name}
    ${${list_name}} ${ARGN}
    )
endfunction(holon_list_append)


# Prepend values to an internally cached variable.
# Usage:
#   holon_list_prepend(HOLON_CORELIB_SOURCE_FILES
#     ${CMAKE_CURRENT_SOURCE_DIR}/e.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/f.cpp
#     )
function(holon_list_prepend list_name)
  holon_set_var(${list_name}
    ${ARGN} ${${list_name}}
    )
endfunction(holon_list_prepend)


# Initialize variables which are required for corelib.
# This is needed to be called at the top of settings for corelib.
# Usage:
#   holon_init_corelib()
function(holon_init_corelib)
  holon_create_var(HOLON_CORELIB_SOURCE_FILES)
  holon_create_var(HOLON_CORELIB_MODULES)
  holon_create_var(HOLON_TEST_SOURCES)
  holon_create_var(HOLON_TEST_MODULES)
endfunction(holon_init_corelib)


# Get module name that is converted to uppercase.
# Usage:
#   holon_get_module_name_uppercase(math module_name)
#     => ${module_name} = MATH
function(holon_get_module_name_uppercase module_name out_variable)
  string(REPLACE "-" "_" module_name_upper "${module_name}")
  string(TOUPPER "${module_name_upper}" module_name_upper)
  set(${out_variable} ${module_name_upper} PARENT_SCOPE)
endfunction(holon_get_module_name_uppercase)


# Clear a list of source files of a specific corelib module.
# Usage:
#   holon_clear_corelib_module_source(math)
#     => ${HOLON_CORELIB_MATH_SOURCE_FILES} = ""
function(holon_clear_corelib_module_source module_name)
  holon_get_module_name_uppercase(${module_name} module)
  holon_create_var(HOLON_CORELIB_${module}_SOURCE_FILES)
endfunction(holon_clear_corelib_module_source)


# Append a source file to a list of a specific corelib module.
# If the source file is relative path, it should be in the current directory.
# Usage:
#   holon_append_corelib_module_source(math vec3d.cpp)
#   holon_append_corelib_module_source(math /path/to/vec3d.cpp)
function(holon_append_corelib_module_source module_name source_file)
  holon_get_module_name_uppercase(${module_name} module)
  if(IS_ABSOLUTE ${source_file})
    holon_list_append(HOLON_CORELIB_${module}_SOURCE_FILES ${source_file})
  else()
    holon_list_append(HOLON_CORELIB_${module}_SOURCE_FILES
      ${CMAKE_CURRENT_SOURCE_DIR}/${source_file}
      )
  endif()
endfunction(holon_append_corelib_module_source)


# Prepend a source file to a list of a specific corelib module.
# If the source file is relative path, it should be in the current directory.
# Usage:
#   holon_prepend_corelib_module_source(math vec3d.cpp)
#   holon_prepend_corelib_module_source(math /path/to/vec3d.cpp)
function(holon_prepend_corelib_module_source module_name source_file)
  holon_get_module_name_uppercase(${module_name} module)
  if(IS_ABSOLUTE ${source_file})
    holon_list_prepend(HOLON_CORELIB_${module}_SOURCE_FILES ${source_file})
  else()
    holon_list_prepend(HOLON_CORELIB_${module}_SOURCE_FILES
      ${CMAKE_CURRENT_SOURCE_DIR}/${source_file}
      )
  endif()
endfunction(holon_prepend_corelib_module_source)


# Add source files to a specific corelib module.
# holon_add_corelib_module should be called beforehand.
# Usage:
#   holon_add_corelib_module_source(
#     math
#     SOURCES vec3d.cpp ode.cpp ...
#     PREPEND   # PREPEND or APPEND
#     )
function(holon_add_corelib_module_source module_name)
  cmake_parse_arguments(THIS "PREPEND;APPEND" "" "SOURCES" ${ARGN})
  foreach(source_file ${THIS_SOURCES})
    if(${THIS_PREPEND})
      holon_prepend_corelib_module_source(${module_name} ${source_file})
    else()
      holon_append_corelib_module_source(${module_name} ${source_file})
    endif()
  endforeach()
endfunction(holon_add_corelib_module_source)


# Add a new corelib module.
# Usage:
#   holon_add_corelib_module(
#     math
#     SOURCES vec3d.cpp ode.cpp ...
#     )
function(holon_add_corelib_module module_name)
  cmake_parse_arguments(THIS "" "" "SOURCES" ${ARGN})
  holon_list_append(HOLON_CORELIB_MODULES ${module_name})
  holon_clear_corelib_module_source(${module_name})
  holon_add_corelib_module_source(${module_name}
    SOURCES ${THIS_SOURCES}
    )
endfunction(holon_add_corelib_module)


# Collect all source files of all modules registered as corelib.
# The collected value is set to HOLON_CORELIB_SOURCE_FILES.
# Usage:
#   holon_collect_corelib_module_source()
function(holon_collect_corelib_module_source)
  foreach(module ${HOLON_CORELIB_MODULES})
    holon_get_module_name_uppercase(${module} module_name)
    holon_list_append(HOLON_CORELIB_SOURCE_FILES
      ${HOLON_CORELIB_${module_name}_SOURCE_FILES}
      )
  endforeach()
endfunction(holon_collect_corelib_module_source)


# Create core library of holon.
# This should be called after all the source files of each module are added.
# Usage:
#   holon_make_corelib()
function(holon_make_corelib)
  holon_collect_corelib_module_source()
  add_library(holon SHARED ${HOLON_CORELIB_SOURCE_FILES})
  target_include_directories(holon PUBLIC ${HOLON_INCLUDE_DIR})
  target_compile_options(holon
    PRIVATE
    $<$<CXX_COMPILER_ID:Clang>:-Wall -Weverything $<$<CONFIG:Debug>:-g3 -O0> $<$<CONFIG:Release>:-O3>>
    $<$<CXX_COMPILER_ID:GNU>:-pedantic -Wall -Wextra -Wshadow -Wnon-virtual-dtor $<$<CONFIG:Debug>:-g3 -O0> $<$<CONFIG:Release>:-O3>>
    $<$<CXX_COMPILER_ID:MSVC>:/W4 $<$<CONFIG:Debug>:/Od> $<$<CONFIG:Release>:/O2>>
    )
  if(NOT TESTING_HOLON2)
    target_link_libraries(holon PUBLIC roki)
  endif()
endfunction(holon_make_corelib)


# Clear a list of source files of a module test.
# Usage:
#   holon_clear_module_test_source(math)
#     => ${HOLON_CORELIB_MATH_TEST_SOURCES} = ""
function(holon_clear_module_test_source module_name)
  holon_get_module_name_uppercase(${module_name} module)
  holon_create_var(HOLON_CORELIB_${module}_TEST_SOURCES)
endfunction(holon_clear_module_test_source)


# Append a source file to a list of a specific module test.
# If the source file is relative path, it should be in the current directory.
# Usage:
#   holon_append_module_test_source(math vec3d_test.cpp)
#   holon_append_module_test_source(math /path/to/vec3d_test.cpp)
function(holon_append_module_test_source module_name test_source)
  holon_get_module_name_uppercase(${module_name} module)
  if(IS_ABSOLUTE ${test_source})
    holon_list_append(HOLON_CORELIB_${module}_TEST_SOURCES ${test_source})
  else()
    holon_list_append(HOLON_CORELIB_${module}_TEST_SOURCES
      ${CMAKE_CURRENT_SOURCE_DIR}/${test_source}
      )
  endif()
endfunction(holon_append_module_test_source)


# Prepend a source file to a list of a specific module test.
# If the source file is relative path, it should be in the current directory.
# Usage:
#   holon_prepend_module_test_source(math vec3d_test.cpp)
#   holon_prepend_module_test_source(math /path/to/vec3d_test.cpp)
function(holon_prepend_module_test_source module_name test_source)
  holon_get_module_name_uppercase(${module_name} module)
  if(IS_ABSOLUTE ${test_source})
    holon_list_prepend(HOLON_CORELIB_${module}_TEST_SOURCES ${test_source})
  else()
    holon_list_prepend(HOLON_CORELIB_${module}_TEST_SOURCES
      ${CMAKE_CURRENT_SOURCE_DIR}/${test_source}
      )
  endif()
endfunction(holon_prepend_module_test_source)


# Add source files to a specific module test.
# holon_add_module_test should be called beforehand.
# Usage:
#   holon_add_module_test_source(
#     math
#     SOURCES vec3d_test.cpp ode_test.cpp ...
#     PREPEND   # PREPEND or APPEND
#     )
function(holon_add_module_test_source module_name)
  cmake_parse_arguments(THIS "PREPEND;APPEND" "" "SOURCES" ${ARGN})
  foreach(test_source ${THIS_SOURCES})
    if(${THIS_PREPEND})
      holon_prepend_module_test_source(${module_name} ${test_source})
    else()
      holon_append_module_test_source(${module_name} ${test_source})
    endif()
  endforeach()
endfunction(holon_add_module_test_source)


# Add a new module test.
# Usage:
#   holon_add_module_test(
#     math
#     SOURCES vec3d_test.cpp ode_test.cpp ...
#     )
function(holon_add_module_test module_name)
  cmake_parse_arguments(THIS "" "" "SOURCES" ${ARGN})
  holon_list_append(HOLON_TEST_MODULES ${module_name})
  holon_clear_module_test_source(${module_name})
  holon_add_module_test_source(${module_name}
    SOURCES ${THIS_SOURCES}
    )
endfunction(holon_add_module_test)


# Create a module test.
# This should be called after all the source files of a module are added.
# Usage:
#   holon_make_module_test(math)
function(holon_make_module_test module_name)
  set(test_name ${module_name}_test)
  holon_get_module_name_uppercase(${module_name} module)
  add_executable(${test_name} ${HOLON_CORELIB_${module}_TEST_SOURCES})
  target_include_directories(${test_name} PUBLIC ${HOLON_INCLUDE_DIR})
  target_link_libraries(${test_name} PUBLIC holon Catch)
  add_test(
    NAME ${test_name}
    COMMAND $<TARGET_FILE:${test_name}> --use-colour yes
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
endfunction(holon_make_module_test module_name)


# Append a source file to a list of common source files to all the tests
# Usage:
#   holon_append_test_source(test_main.cpp)
function(holon_append_test_source test_source)
  if(IS_ABSOLUTE ${test_source})
    holon_list_append(HOLON_TEST_SOURCES ${test_source})
  else()
    holon_list_append(HOLON_TEST_SOURCES
      ${CMAKE_CURRENT_SOURCE_DIR}/${test_source}
      )
  endif()
endfunction(holon_append_test_source)


# Prepend a source file to a list of common source files to all the tests
# Usage:
#   holon_prepend_test_source(test_main.cpp)
function(holon_prepend_test_source test_source)
  if(IS_ABSOLUTE ${test_source})
    holon_list_prepend(HOLON_TEST_SOURCES ${test_source})
  else()
    holon_list_prepend(HOLON_TEST_SOURCES
      ${CMAKE_CURRENT_SOURCE_DIR}/${test_source}
      )
  endif()
endfunction(holon_prepend_test_source)


# Add source files to a list of source files that are used by all the module tests
# Usage:
#   holon_add_test_source(
#     SOURCES test_main.cpp
#     APPEND                 # PREPEND or APPEND
#     )
function(holon_add_test_source)
  cmake_parse_arguments(THIS "PREPEND;APPEND" "" "SOURCES" ${ARGN})
  foreach(test_source ${THIS_SOURCES})
    if(${THIS_PREPEND})
      holon_prepend_test_source(${test_source})
    else()
      holon_append_test_source(${test_source})
    endif()
  endforeach()
endfunction(holon_add_test_source)


# Create all the module tests.
# This should be called after all the module tests are registered
# and all the common source files are added.
# Usage:
#   holon_make_all_tests()
function(holon_make_all_tests)
  foreach(module ${HOLON_TEST_MODULES})
    holon_add_module_test_source(${module}
      APPEND
      SOURCES ${HOLON_TEST_SOURCES})
    holon_make_module_test(${module})
  endforeach()
endfunction(holon_make_all_tests)


# Add a new example.
# Usage:
#   holon_add_example(new_example.cpp)
function(holon_add_example source)
  if(NOT IS_ABSOLUTE ${source})
    set(source ${CMAKE_CURRENT_SOURCE_DIR}/${source})
  endif()
  get_filename_component(target ${source} NAME_WE)
  add_executable(${target} ${source})
  target_compile_options(${target}
    PRIVATE
    $<$<CXX_COMPILER_ID:Clang>:-Wall -Weverything $<$<CONFIG:Debug>:-g3 -O0> $<$<CONFIG:Release>:-O3>>
    $<$<CXX_COMPILER_ID:GNU>:-pedantic -Wall -Wextra -Wshadow -Wnon-virtual-dtor $<$<CONFIG:Debug>:-g3 -O0> $<$<CONFIG:Release>:-O3>>
    $<$<CXX_COMPILER_ID:MSVC>:/W4 $<$<CONFIG:Debug>:/Od> $<$<CONFIG:Release>:/O2>>
    )
  target_link_libraries(${target} PUBLIC holon)
endfunction(holon_add_example)


# Add a new script for testing.
# Usage:
#   holon_add_test_script(new_example_test.py)
function(holon_add_test_script ${script})
  if(NOT IS_ABSOLUTE ${script})
    set(script ${CMAKE_CURRENT_SOURCE_DIR}/${script})
  endif()
  get_filename_component(filename ${script} NAME)
  get_filename_component(testname ${script} NAME_WE)
  set(output ${CMAKE_CURRENT_BINARY_DIR}/${filename})
  add_custom_command(
    OUTPUT ${output}
    COMMAND ${CMAKE_COMMAND} -E copy ${script} ${output}
    DEPENDS ${script}
    )
  add_custom_target(
    ${filename} ALL
    DEPENDS ${output}
    )
  if(${PYTHON_NUMPY_FOUND})
    add_test(
      NAME ${testname}
      COMMAND ${PYTHON_EXECUTABLE} ${filename}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      )
  endif()
endfunction(holon_add_test_script)
