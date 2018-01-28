cmake_minimum_required(VERSION 3.2)
project(liw C)

# set locations
set(LIW_TOP_DIR     ${PROJECT_SOURCE_DIR}/liw)
set(LIW_SOURCE_DIR  ${LIW_TOP_DIR}/src)
set(LIW_INCLUDE_DIR ${LIW_TOP_DIR}/include)
set(LIW_LIBRARY_DIR ${LIW_TOP_DIR}/lib)
set(LIW_APP_DIR     ${LIW_TOP_DIR}/app)

# set options
option(LIW_USE_LIBDL "Determines whether to use dynamic link library." ON)
option(LIW_USE_PTHREAD "Determines whether to enable POSIX thread programming." ON)
option(LIW_USE_RT "Determines whether to use realtime extension." ON)
option(LIW_USE_RTC "Determines whether to use enhanced realtime controller (RTC)." ON)

# get version number
file(READ ${LIW_TOP_DIR}/config.org LIW_CONFIG_CONTENT)
string(REGEX MATCH
  "PREFIX=(.*)[\r\n\t ]PROJNAME=(.*)[\r\n\t ]VERSION=(.*)[\r\n\t ][\r\n\t ]#.*#.*#.*#.*"
  _ ${LIW_CONFIG_CONTENT})
set(LIW_PREFIX   ${CMAKE_MATCH_1})
set(LIW_PROJNAME ${CMAKE_MATCH_2})
set(LIW_VERSION  ${CMAKE_MATCH_3})
# message(STATUS "prefix:" ${LIW_PREFIX})
# message(STATUS "projname:" ${LIW_PROJNAME})
# message(STATUS "version:" ${LIW_VERSION})

# build library
set(LIW_SOURCE_FILES
  ${LIW_SOURCE_DIR}/liw_time.c
  ${LIW_SOURCE_DIR}/liw_coproc.c
  ${LIW_SOURCE_DIR}/liw_paction.c
  ${LIW_SOURCE_DIR}/liw_fifo.c
  ${LIW_SOURCE_DIR}/liw_ipc.c
  ${LIW_SOURCE_DIR}/liw_lock.c
  ${LIW_SOURCE_DIR}/liw_socket.c
  )
if(LIW_USE_LIBDL)
  list(APPEND LIW_SOURCE_FILES ${LIW_SOURCE_DIR}/liw_dl.c)
endif()
if(LIW_USE_RTC)
  list(APPEND LIW_SOURCE_FILES ${LIW_SOURCE_DIR}/liw_rtc.c)
endif()
add_library(liw SHARED ${LIW_SOURCE_FILES})
target_include_directories(liw PUBLIC ${LIW_INCLUDE_DIR})
target_link_libraries(liw cure)

# build app
set(LIW_APPS)
foreach(app ${LIW_APPS})
  add_executable(${app} ${LIW_APP_DIR}/${app}.c)
  target_link_libraries(${app} liw)
  if(LIW_USE_LIBDL)
    target_link_libraries(${app} dl)
  endif()
  if(LIW_USE_PTHREAD)
    target_link_libraries(${app} pthread)
  endif()
  if(LIW_USE_RT)
    target_link_libraries(${app} rt)
  endif()
endforeach()
if(UNIX)
  execute_process(
    COMMAND cure-config -l
    OUTPUT_VARIABLE CURE_LINK
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  set(LIW_LINK "-lliw ${CURE_LINK}")
  add_custom_command(
    OUTPUT liw-config
    COMMAND sh ${CMAKE_BINARY_DIR}/cure-config-gen -I ${CMAKE_INSTALL_PREFIX}/include -L ${CMAKE_INSTALL_PREFIX}/lib -l ${LIW_LINK} -v ${LIW_VERSION} > liw-config
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
  add_custom_target(
    make-liw-config ALL
    DEPENDS make-cure-config-gen
    SOURCES liw-config
    )
endif()

# install
set(PERMISSIONS_755
  OWNER_READ OWNER_WRITE OWNER_EXECUTE
  GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE
  )
install(TARGETS liw
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
install(DIRECTORY ${LIW_INCLUDE_DIR}/ DESTINATION include)
install(TARGETS ${LIW_APPS}
  RUNTIME DESTINATION bin
  )
if(UNIX)
  install(FILES ${CMAKE_BINARY_DIR}/liw-config
    PERMISSIONS ${PERMISSIONS_755}
    DESTINATION bin
    )
endif()
