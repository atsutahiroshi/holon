cmake_minimum_required(VERSION 3.2)
project(roki C)

# set locations
set(ROKI_TOP_DIR     ${PROJECT_SOURCE_DIR}/roki)
set(ROKI_SOURCE_DIR  ${ROKI_TOP_DIR}/src)
set(ROKI_INCLUDE_DIR ${ROKI_TOP_DIR}/include)
set(ROKI_LIBRARY_DIR ${ROKI_TOP_DIR}/lib)
set(ROKI_APP_DIR     ${ROKI_TOP_DIR}/app)

# get version number
file(READ ${ROKI_TOP_DIR}/config.org ROKI_CONFIG_CONTENT)
string(REGEX MATCH
  "PREFIX=(.*)[\r\n\t ]PROJNAME=(.*)[\r\n\t ]VERSION=(.*)[\r\n\t ]"
  _ ${ROKI_CONFIG_CONTENT})
set(ROKI_PREFIX   ${CMAKE_MATCH_1})
set(ROKI_PROJNAME ${CMAKE_MATCH_2})
set(ROKI_VERSION  ${CMAKE_MATCH_3})
# message(STATUS "prefix:" ${ROKI_PREFIX})
# message(STATUS "projname:" ${ROKI_PROJNAME})
# message(STATUS "version:" ${ROKI_VERSION})

# build library
set(ROKI_SOURCE_FILES
  ${ROKI_SOURCE_DIR}/rk_g3d.c
  ${ROKI_SOURCE_DIR}/rk_contact.c
  ${ROKI_SOURCE_DIR}/rk_motor.c
  ${ROKI_SOURCE_DIR}/rk_motor_none.c
  ${ROKI_SOURCE_DIR}/rk_motor_trq.c
  ${ROKI_SOURCE_DIR}/rk_motor_dc.c
  ${ROKI_SOURCE_DIR}/rk_force.c
  ${ROKI_SOURCE_DIR}/rk_body.c
  ${ROKI_SOURCE_DIR}/rk_joint.c
  ${ROKI_SOURCE_DIR}/rk_joint_fixed.c
  ${ROKI_SOURCE_DIR}/rk_joint_revol.c
  ${ROKI_SOURCE_DIR}/rk_joint_prism.c
  ${ROKI_SOURCE_DIR}/rk_joint_cylin.c
  ${ROKI_SOURCE_DIR}/rk_joint_hooke.c
  ${ROKI_SOURCE_DIR}/rk_joint_spher.c
  ${ROKI_SOURCE_DIR}/rk_joint_float.c
  ${ROKI_SOURCE_DIR}/rk_link.c
  ${ROKI_SOURCE_DIR}/rk_chain.c
  ${ROKI_SOURCE_DIR}/rk_jacobi.c
  ${ROKI_SOURCE_DIR}/rk_ik_cell.c
  ${ROKI_SOURCE_DIR}/rk_ik.c
  ${ROKI_SOURCE_DIR}/rk_ik_seq.c
  ${ROKI_SOURCE_DIR}/rk_ik_imp.c
  ${ROKI_SOURCE_DIR}/rk_cd.c
  ${ROKI_SOURCE_DIR}/rk_abi.c
  ${ROKI_SOURCE_DIR}/rk_fd.c
  )
add_library(roki SHARED ${ROKI_SOURCE_FILES})
target_include_directories(roki SYSTEM PUBLIC ${ROKI_INCLUDE_DIR})
target_link_libraries(roki PUBLIC zeo zm cure)

# build app
set(ROKI_APPS
  cad2zkc zkcconv z3d2zkc zkc2z3d z3d2gp zvs2zkci zvs2zkcs
  rk_ground rk_mp rk_fk rk_ik rk_id rk_ikcell_merge
  )
foreach(app ${ROKI_APPS})
  add_executable(${app} ${ROKI_APP_DIR}/${app}.c)
  target_link_libraries(${app} roki)
endforeach()
if(UNIX)
  execute_process(
    COMMAND zeo-config -l
    OUTPUT_VARIABLE ZEO_LINK
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  set(ROKI_LINK "-lroki ${ZEO_LINK}")
  add_custom_command(
    OUTPUT roki-config
    COMMAND sh ${CMAKE_BINARY_DIR}/cure-config-gen -I ${CMAKE_INSTALL_PREFIX}/include -L ${CMAKE_INSTALL_PREFIX}/lib -l ${ROKI_LINK} -v ${ROKI_VERSION} > roki-config
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
  add_custom_target(
    make-roki-config ALL
    DEPENDS make-cure-config-gen
    SOURCES roki-config
    )
endif()

# install
set(PERMISSIONS_755
  OWNER_READ OWNER_WRITE OWNER_EXECUTE
  GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE
  )
install(TARGETS roki
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
install(DIRECTORY ${ROKI_INCLUDE_DIR}/ DESTINATION include)
install(TARGETS ${ROKI_APPS}
  RUNTIME DESTINATION bin
  )
if(UNIX)
  install(FILES ${CMAKE_BINARY_DIR}/roki-config
    PERMISSIONS ${PERMISSIONS_755}
    DESTINATION bin
    )
endif()
