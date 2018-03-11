include(CMakeParseArguments)

function(holon_create_var var_name)
  set(${var_name} ""
    CACHE INTERNAL ""
    )
endfunction(holon_create_var)

function(holon_set_var var_name)
  set(${var_name} ${ARGN}
    CACHE INTERNAL ""
    )
endfunction(holon_set_var)

function(holon_list_append list_name)
  holon_set_var(${list_name}
    ${${list_name}} ${ARGN}
    )
endfunction(holon_list_append)

function(holon_list_prepend list_name)
  holon_set_var(${list_name}
    ${ARGN} ${${list_name}}
    )
endfunction(holon_list_prepend)

function(holon_init_corelib)
  holon_create_var(HOLON_CORELIB_SOURCE_FILES)
  holon_create_var(HOLON_CORELIB_MODULES)
  holon_create_var(HOLON_TEST_SOURCES)
  holon_create_var(HOLON_TEST_MODULES)
endfunction(holon_init_corelib)

function(holon_get_module_name_uppercase module_name out_variable)
  string(REPLACE "-" "_" module_name_upper "${module_name}")
  string(TOUPPER "${module_name_upper}" module_name_upper)
  set(${out_variable} ${module_name_upper} PARENT_SCOPE)
endfunction(holon_get_module_name_uppercase)

function(holon_clear_corelib_module_source module_name)
  holon_get_module_name_uppercase(${module_name} module)
  holon_create_var(HOLON_CORELIB_${module}_SOURCE_FILES)
endfunction(holon_clear_corelib_module_source)

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

function(holon_add_corelib_module module_name)
  cmake_parse_arguments(THIS "" "" "SOURCES" ${ARGN})
  holon_list_append(HOLON_CORELIB_MODULES ${module_name})
  holon_clear_corelib_module_source(${module_name})
  holon_add_corelib_module_source(${module_name}
    SOURCES ${THIS_SOURCES}
    )
endfunction(holon_add_corelib_module)

function(holon_collect_corelib_module_source)
  foreach(module ${HOLON_CORELIB_MODULES})
    holon_get_module_name_uppercase(${module} module_name)
    holon_list_append(HOLON_CORELIB_SOURCE_FILES
      ${HOLON_CORELIB_${module_name}_SOURCE_FILES}
      )
  endforeach()
endfunction(holon_collect_corelib_module_source)

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
  target_link_libraries(holon PUBLIC roki)
endfunction(holon_make_corelib)


function(holon_clear_module_test_source module_name)
  holon_get_module_name_uppercase(${module_name} module)
  holon_create_var(HOLON_CORELIB_${module}_TEST_SOURCES)
endfunction(holon_clear_module_test_source)

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

function(holon_add_module_test module_name)
  cmake_parse_arguments(THIS "" "" "SOURCES" ${ARGN})
  holon_list_append(HOLON_TEST_MODULES ${module_name})
  holon_clear_module_test_source(${module_name})
  holon_add_module_test_source(${module_name}
    SOURCES ${THIS_SOURCES}
    )
endfunction(holon_add_module_test)

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

function(holon_append_test_source test_source)
  if(IS_ABSOLUTE ${test_source})
    holon_list_append(HOLON_TEST_SOURCES ${test_source})
  else()
    holon_list_append(HOLON_TEST_SOURCES
      ${CMAKE_CURRENT_SOURCE_DIR}/${test_source}
      )
  endif()
endfunction(holon_append_test_source)

function(holon_prepend_test_source test_source)
  if(IS_ABSOLUTE ${test_source})
    holon_list_prepend(HOLON_TEST_SOURCES ${test_source})
  else()
    holon_list_prepend(HOLON_TEST_SOURCES
      ${CMAKE_CURRENT_SOURCE_DIR}/${test_source}
      )
  endif()
endfunction(holon_prepend_test_source)

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

function(holon_make_all_tests)
  foreach(module ${HOLON_TEST_MODULES})
    holon_add_module_test_source(${module}
      APPEND
      SOURCES ${HOLON_TEST_SOURCES})
    holon_make_module_test(${module})
  endforeach()
endfunction(holon_make_all_tests)


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
