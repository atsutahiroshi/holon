cmake_minimum_required(VERSION 3.2)
project(cure C)

# set locations
set(CURE_TOP_DIR     ${PROJECT_SOURCE_DIR}/cure)
set(CURE_SOURCE_DIR  ${CURE_TOP_DIR}/src)
set(CURE_INCLUDE_DIR ${CURE_TOP_DIR}/include)
set(CURE_LIBRARY_DIR ${CURE_TOP_DIR}/lib)
set(CURE_APP_DIR     ${CURE_TOP_DIR}/app)

# get version number
file(READ ${CURE_TOP_DIR}/config.org CURE_CONFIG_CONTENT)
string(REGEX MATCH
  "PREFIX=(.*)[\r\n\t ]PROJNAME=(.*)[\r\n\t ]VERSION=(.*)[\r\n\t ]"
  _ ${CURE_CONFIG_CONTENT})
set(CURE_PREFIX   ${CMAKE_MATCH_1})
set(CURE_PROJNAME ${CMAKE_MATCH_2})
set(CURE_VERSION  ${CMAKE_MATCH_3})
# message(STATUS "prefix:" ${CURE_PREFIX})
# message(STATUS "projname:" ${CURE_PROJNAME})
# message(STATUS "version:" ${CURE_VERSION})

# build library
set(CURE_SOURCE_FILES
  ${CURE_SOURCE_DIR}/cure_misc.c
  ${CURE_SOURCE_DIR}/cure_string.c
  ${CURE_SOURCE_DIR}/cure_bit.c
  ${CURE_SOURCE_DIR}/cure_rand.c
  ${CURE_SOURCE_DIR}/cure_index.c
  ${CURE_SOURCE_DIR}/cure_stream.c
  ${CURE_SOURCE_DIR}/cure_list.c
  ${CURE_SOURCE_DIR}/cure_rrtab.c
  ${CURE_SOURCE_DIR}/cure_strlist.c
  ${CURE_SOURCE_DIR}/cure_option.c
  )
add_library(cure SHARED ${CURE_SOURCE_FILES})
target_include_directories(cure SYSTEM PUBLIC ${CURE_INCLUDE_DIR})
target_link_libraries(cure PUBLIC m)

# build app
set(CURE_LINKER_FLAGS "-lcure -lm")
if(UNIX)
  add_custom_command(
    OUTPUT cure-config-gen
    COMMAND cp ${CURE_APP_DIR}/cure-config-gen ${CMAKE_CURRENT_BINARY_DIR}
    )
  add_custom_command(
    OUTPUT cure-config
    COMMAND sh ${CMAKE_CURRENT_BINARY_DIR}/cure-config-gen -I ${CMAKE_INSTALL_PREFIX}/include -L ${CMAKE_INSTALL_PREFIX}/lib -l ${CURE_LINKER_FLAGS} -v ${CURE_VERSION} > cure-config
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
  add_custom_target(
    make-cure-config-gen ALL
    SOURCES cure-config-gen
    )
  add_custom_target(
    make-cure-config ALL
    DEPENDS make-cure-config-gen
    SOURCES cure-config
    )
endif()

# install
set(PERMISSIONS_755
  OWNER_READ OWNER_WRITE OWNER_EXECUTE
  GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE
  )
install(TARGETS cure
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
install(DIRECTORY ${CURE_INCLUDE_DIR}/ DESTINATION include)
if(UNIX)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cure-config-gen
    PERMISSIONS ${PERMISSIONS_755}
    DESTINATION bin
    )
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cure-config
    PERMISSIONS ${PERMISSIONS_755}
    DESTINATION bin
    )
endif()
