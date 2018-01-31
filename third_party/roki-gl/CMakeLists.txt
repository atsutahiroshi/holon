cmake_minimum_required(VERSION 3.2)
project(rokigl C)

# set locations
set(ROKIGL_TOP_DIR     ${PROJECT_SOURCE_DIR}/roki-gl)
set(ROKIGL_SOURCE_DIR  ${ROKIGL_TOP_DIR}/src)
set(ROKIGL_INCLUDE_DIR ${ROKIGL_TOP_DIR}/include)
set(ROKIGL_LIBRARY_DIR ${ROKIGL_TOP_DIR}/lib)
set(ROKIGL_APP_DIR     ${ROKIGL_TOP_DIR}/app)

# set options
option(ROKIGL_USE_GLX "Determines whther to use GLX." ON)
option(ROKIGL_USE_GLUT "Determines whther to use GLUT." ON)

# get version number
file(READ ${ROKIGL_TOP_DIR}/config.org ROKIGL_CONFIG_CONTENT)
string(REGEX MATCH
  "PREFIX=(.*)[\r\n\t ]PROJNAME=(.*)[\r\n\t ]VERSION=([0-9beta\\.\\-]*)"
  _ ${ROKIGL_CONFIG_CONTENT})
set(ROKIGL_PREFIX   ${CMAKE_MATCH_1})
set(ROKIGL_PROJNAME ${CMAKE_MATCH_2})
set(ROKIGL_VERSION  ${CMAKE_MATCH_3})
# message(STATUS "prefix:" ${ROKIGL_PREFIX})
# message(STATUS "projname:" ${ROKIGL_PROJNAME})
# message(STATUS "version:" ${ROKIGL_VERSION})

# build library
set(ROKIGL_SOURCE_FILES
  ${ROKIGL_SOURCE_DIR}/rkgl_misc.c
  ${ROKIGL_SOURCE_DIR}/rkgl_camera.c
  ${ROKIGL_SOURCE_DIR}/rkgl_optic.c
  ${ROKIGL_SOURCE_DIR}/rkgl_shape.c
  ${ROKIGL_SOURCE_DIR}/rkgl_chain.c
  ${ROKIGL_SOURCE_DIR}/rkgl_shadow.c
  ${ROKIGL_SOURCE_DIR}/rkgl_select.c
  )
add_library(rokigl SHARED ${ROKIGL_SOURCE_FILES})
target_include_directories(rokigl SYSTEM PUBLIC ${ROKIGL_INCLUDE_DIR})

# specify build options
set(ROKIGL_LINKER_FLAGS "-lrokigl") # used for generating rokigl-config
if(ROKIGL_USE_GLX)
  target_sources(rokigl PRIVATE ${ROKIGL_SOURCE_DIR}/rkgl_glx.c)
endif()
if(ROKIGL_USE_GLUT)
  target_sources(rokigl PRIVATE ${ROKIGL_SOURCE_DIR}/rkgl_glut.c)
endif()
set(ROKIGL_LINKER_FLAGS "${ROKIGL_LINKER_FLAGS} -lGLEW -lglut -lGLU -lGL")
target_link_libraries(rokigl PUBLIC GLEW glut GLU GL zx11 liw roki)

# build app
set(ROKIGL_APPS rk_view rk_anim rk_seq rk_pen)
foreach(app ${ROKIGL_APPS})
  add_executable(${app} ${ROKIGL_APP_DIR}/${app}.c)
  target_compile_definitions(${app} PRIVATE __USE_GLEW)
  target_link_libraries(${app} rokigl)
endforeach()
if(UNIX)
  add_custom_command(
    OUTPUT rokigl-config
    COMMAND echo -n "${ROKIGL_LINKER_FLAGS} " > rokigl_link
    COMMAND sh $<TARGET_FILE_DIR:zx11>/zx11-config -l >> rokigl_link
    COMMAND sh $<TARGET_FILE_DIR:liw>/liw-config -l >> rokigl_link
    COMMAND sh $<TARGET_FILE_DIR:roki>/roki-config -l >> rokigl_link
    COMMAND sh $<TARGET_FILE_DIR:cure>/cure-config-gen -I ${CMAKE_INSTALL_PREFIX}/include -L ${CMAKE_INSTALL_PREFIX}/lib -l \"`cat rokigl_link | tr \'\\n\' \'\ \'`\" -v ${ROKIGL_VERSION} > rokigl-config
    COMMAND rm -f rokigl_link
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
  add_custom_target(
    make-rokigl-config ALL
    DEPENDS make-cure-config-gen make-zx11-config make-liw-config make-roki-config
    SOURCES rokigl-config
    )
endif()

# install
set(PERMISSIONS_755
  OWNER_READ OWNER_WRITE OWNER_EXECUTE
  GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE
  )
install(TARGETS rokigl
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
install(DIRECTORY ${ROKIGL_INCLUDE_DIR}/ DESTINATION include)
install(TARGETS ${ROKIGL_APPS}
  RUNTIME DESTINATION bin
  )
if(UNIX)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/rokigl-config
    PERMISSIONS ${PERMISSIONS_755}
    DESTINATION bin
    )
endif()
