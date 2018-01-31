cmake_minimum_required(VERSION 3.2)
project(dzco C)

# set locations
set(DZCO_TOP_DIR     ${PROJECT_SOURCE_DIR}/dzco)
set(DZCO_SOURCE_DIR  ${DZCO_TOP_DIR}/src)
set(DZCO_INCLUDE_DIR ${DZCO_TOP_DIR}/include)
set(DZCO_LIBRARY_DIR ${DZCO_TOP_DIR}/lib)
set(DZCO_APP_DIR     ${DZCO_TOP_DIR}/app)

# get version number
file(READ ${DZCO_TOP_DIR}/config.org DZCO_CONFIG_CONTENT)
string(REGEX MATCH
  "PREFIX=(.*)[\r\n\t ]PROJNAME=(.*)[\r\n\t ]VERSION=(.*)[\r\n\t ]"
  _ ${DZCO_CONFIG_CONTENT})
set(DZCO_PREFIX   ${CMAKE_MATCH_1})
set(DZCO_PROJNAME ${CMAKE_MATCH_2})
set(DZCO_VERSION  ${CMAKE_MATCH_3})
# message(STATUS "prefix:" ${DZCO_PREFIX})
# message(STATUS "projname:" ${DZCO_PROJNAME})
# message(STATUS "version:" ${DZCO_VERSION})

# build library
set(DZCO_SOURCE_FILES
  ${DZCO_SOURCE_DIR}/dz_pex.c
  ${DZCO_SOURCE_DIR}/dz_lin.c
  ${DZCO_SOURCE_DIR}/dz_sys.c
  ${DZCO_SOURCE_DIR}/dz_sys_misc.c
  ${DZCO_SOURCE_DIR}/dz_sys_pid.c
  ${DZCO_SOURCE_DIR}/dz_sys_lag.c
  ${DZCO_SOURCE_DIR}/dz_sys_lin.c
  ${DZCO_SOURCE_DIR}/dz_sys_pex.c
  ${DZCO_SOURCE_DIR}/dz_sys_filt_maf.c
  ${DZCO_SOURCE_DIR}/dz_sys_filt_bw.c
  ${DZCO_SOURCE_DIR}/dz_sys_gen.c
  ${DZCO_SOURCE_DIR}/dz_ident_lag.c
  )
add_library(dzco SHARED ${DZCO_SOURCE_FILES})
target_include_directories(dzco SYSTEM PUBLIC ${DZCO_INCLUDE_DIR})
target_link_libraries(dzco PUBLIC zm cure)

# build app
set(DZCO_APPS dz_bode dz_nyquist dz_sim)
foreach(app ${DZCO_APPS})
  add_executable(${app} ${DZCO_APP_DIR}/${app}.c)
  target_link_libraries(${app} dzco)
endforeach()
set(DZCO_LINKER_FLAGS "-ldzco")
if(UNIX)
  add_custom_command(
    OUTPUT dzco-config
    COMMAND echo -n "${DZCO_LINKER_FLAGS} " > dzco_link
    COMMAND sh $<TARGET_FILE_DIR:zm>/zm-config -l >> dzco_link
    COMMAND sh $<TARGET_FILE_DIR:cure>/cure-config-gen -I ${CMAKE_INSTALL_PREFIX}/include -L ${CMAKE_INSTALL_PREFIX}/lib -l \"`cat dzco_link`\" -v ${DZCO_VERSION} > dzco-config
    COMMAND rm -f dzco_link
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
  add_custom_target(
    make-dzco-config ALL
    DEPENDS make-cure-config-gen make-zm-config
    SOURCES dzco-config
    )
endif()

# install
set(PERMISSIONS_755
  OWNER_READ OWNER_WRITE OWNER_EXECUTE
  GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE
  )
install(TARGETS dzco
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
install(DIRECTORY ${DZCO_INCLUDE_DIR}/ DESTINATION include)
install(TARGETS ${DZCO_APPS}
  RUNTIME DESTINATION bin
  )
if(UNIX)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/dzco-config
    PERMISSIONS ${PERMISSIONS_755}
    DESTINATION bin
    )
endif()
