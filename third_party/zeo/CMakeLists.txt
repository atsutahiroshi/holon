cmake_minimum_required(VERSION 3.2)
project(zeo C)

# set locations
set(ZEO_TOP_DIR     ${PROJECT_SOURCE_DIR}/zeo)
set(ZEO_SOURCE_DIR  ${ZEO_TOP_DIR}/src)
set(ZEO_INCLUDE_DIR ${ZEO_TOP_DIR}/include)
set(ZEO_LIBRARY_DIR ${ZEO_TOP_DIR}/lib)
set(ZEO_APP_DIR     ${ZEO_TOP_DIR}/app)

# get version number
file(READ ${ZEO_TOP_DIR}/config.org ZEO_CONFIG_CONTENT)
string(REGEX MATCH
  "PREFIX=(.*)[\r\n\t ]PROJNAME=(.*)[\r\n\t ]VERSION=(.*)[\r\n\t ]"
  _ ${ZEO_CONFIG_CONTENT})
set(ZEO_PREFIX   ${CMAKE_MATCH_1})
set(ZEO_PROJNAME ${CMAKE_MATCH_2})
set(ZEO_VERSION  ${CMAKE_MATCH_3})
# message(STATUS "prefix:" ${ZEO_PREFIX})
# message(STATUS "projname:" ${ZEO_PROJNAME})
# message(STATUS "version:" ${ZEO_VERSION})

# build library
set(ZEO_SOURCE_FILES
  ${ZEO_SOURCE_DIR}/zeo_color.c
  ${ZEO_SOURCE_DIR}/zeo_optic.c
  ${ZEO_SOURCE_DIR}/zeo_misc.c
  ${ZEO_SOURCE_DIR}/zeo_vec2d.c
  ${ZEO_SOURCE_DIR}/zeo_mat2d.c
  ${ZEO_SOURCE_DIR}/zeo_vec3d.c
  ${ZEO_SOURCE_DIR}/zeo_vec6d.c
  ${ZEO_SOURCE_DIR}/zeo_mat3d.c
  ${ZEO_SOURCE_DIR}/zeo_mat6d.c
  ${ZEO_SOURCE_DIR}/zeo_vec3d_list.c
  ${ZEO_SOURCE_DIR}/zeo_vec3d_tree.c
  ${ZEO_SOURCE_DIR}/zeo_vec3d_pca.c
  ${ZEO_SOURCE_DIR}/zeo_ep.c
  ${ZEO_SOURCE_DIR}/zeo_frame.c
  ${ZEO_SOURCE_DIR}/zeo_elem.c
  ${ZEO_SOURCE_DIR}/zeo_elem_list.c
  ${ZEO_SOURCE_DIR}/zeo_ph.c
  ${ZEO_SOURCE_DIR}/zeo_triangulate.c
  ${ZEO_SOURCE_DIR}/zeo_prim_box.c
  ${ZEO_SOURCE_DIR}/zeo_prim_sphere.c
  ${ZEO_SOURCE_DIR}/zeo_prim_ellips.c
  ${ZEO_SOURCE_DIR}/zeo_prim_cyl.c
  ${ZEO_SOURCE_DIR}/zeo_prim_ecyl.c
  ${ZEO_SOURCE_DIR}/zeo_prim_cone.c
  ${ZEO_SOURCE_DIR}/zeo_prim_ph.c
  ${ZEO_SOURCE_DIR}/zeo_shape.c
  ${ZEO_SOURCE_DIR}/zeo_shape_list.c
  ${ZEO_SOURCE_DIR}/zeo_mshape.c
  ${ZEO_SOURCE_DIR}/zeo_brep.c
  ${ZEO_SOURCE_DIR}/zeo_brep_trunc.c
  ${ZEO_SOURCE_DIR}/zeo_brep_bool.c
  ${ZEO_SOURCE_DIR}/zeo_bv_ch2.c
  ${ZEO_SOURCE_DIR}/zeo_bv_aabb.c
  ${ZEO_SOURCE_DIR}/zeo_bv_obb.c
  ${ZEO_SOURCE_DIR}/zeo_bv_bball.c
  ${ZEO_SOURCE_DIR}/zeo_bv_qhull.c
  ${ZEO_SOURCE_DIR}/zeo_col.c
  ${ZEO_SOURCE_DIR}/zeo_col_box.c
  ${ZEO_SOURCE_DIR}/zeo_col_aabbtree.c
  ${ZEO_SOURCE_DIR}/zeo_col_minkowski.c
  ${ZEO_SOURCE_DIR}/zeo_col_gjk.c
  ${ZEO_SOURCE_DIR}/zeo_col_ph.c
  )
add_library(zeo SHARED ${ZEO_SOURCE_FILES})
target_include_directories(zeo SYSTEM PUBLIC ${ZEO_INCLUDE_DIR})
if(UNIX)
  target_compile_options(zeo
    PRIVATE
    -ansi -Wall
    $<$<CONFIG:Debug>:-g3 -O0>
    $<$<CONFIG:Release>:-O3 -funroll-loops>
    )
elseif(MSVC)
  target_compile_options(zeo
    PRIVATE
    /W4
    $<$<CONFIG:Debug>:/Od>
    $<$<CONFIG:Release>:/O2>
    )
endif()
target_link_libraries(zeo PUBLIC zm cure)

# build app
set(ZEO_APPS qhull gjk terra)
foreach(app ${ZEO_APPS})
  add_executable(${app} ${ZEO_APP_DIR}/${app}.c)
  if(UNIX)
    target_compile_options(${app}
      PRIVATE
      -ansi -Wall
      $<$<CONFIG:Debug>:-g3 -O0>
      $<$<CONFIG:Release>:-O3>
      )
  elseif(MSVC)
    target_compile_options(${app}
      PRIVATE
      /W4
      $<$<CONFIG:Debug>:/Od>
      $<$<CONFIG:Release>:/O2>
      )
  endif()
  target_link_libraries(${app} zeo)
endforeach()
set(ZEO_LINKER_FLAGS "-lzeo")
if(UNIX)
  add_custom_command(
    OUTPUT zeo-config
    COMMAND echo -n "${ZEO_LINKER_FLAGS} " > zeo_link
    COMMAND sh $<TARGET_FILE_DIR:zm>/zm-config -l >> zeo_link
    COMMAND sh $<TARGET_FILE_DIR:cure>/cure-config-gen -I ${CMAKE_INSTALL_PREFIX}/include -L ${CMAKE_INSTALL_PREFIX}/lib -l \"`cat zeo_link`\" -v ${ZEO_VERSION} > zeo-config
    COMMAND rm -f zeo_link
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
  add_custom_target(
    make-zeo-config ALL
    DEPENDS make-cure-config-gen make-zm-config
    SOURCES zeo-config
    )
endif()

# install
set(PERMISSIONS_755
  OWNER_READ OWNER_WRITE OWNER_EXECUTE
  GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE
  )
install(TARGETS zeo
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
install(DIRECTORY ${ZEO_INCLUDE_DIR}/ DESTINATION include)
install(TARGETS ${ZEO_APPS}
  RUNTIME DESTINATION bin
  )
if(UNIX)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/zeo-config
    PERMISSIONS ${PERMISSIONS_755}
    DESTINATION bin
    )
endif()
