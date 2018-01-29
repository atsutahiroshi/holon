cmake_minimum_required(VERSION 3.2)
project(zm C)

# set locations
set(ZM_TOP_DIR     ${PROJECT_SOURCE_DIR}/zm)
set(ZM_SOURCE_DIR  ${ZM_TOP_DIR}/src)
set(ZM_INCLUDE_DIR ${ZM_TOP_DIR}/include)
set(ZM_LIBRARY_DIR ${ZM_TOP_DIR}/lib)
set(ZM_APP_DIR     ${ZM_TOP_DIR}/app)

# get version number
file(READ ${ZM_TOP_DIR}/config.org ZM_CONFIG_CONTENT)
string(REGEX MATCH
  "PREFIX=(.*)[\r\n\t ]PROJNAME=(.*)[\r\n\t ]VERSION=(.*)[\r\n\t ]"
  _ ${ZM_CONFIG_CONTENT})
set(ZM_PREFIX   ${CMAKE_MATCH_1})
set(ZM_PROJNAME ${CMAKE_MATCH_2})
set(ZM_VERSION  ${CMAKE_MATCH_3})
# message(STATUS "prefix:" ${ZM_PREFIX})
# message(STATUS "projname:" ${ZM_PROJNAME})
# message(STATUS "version:" ${ZM_VERSION})

# build library
set(ZM_SOURCE_FILES
  ${ZM_SOURCE_DIR}/zm_ieee.c
  ${ZM_SOURCE_DIR}/zm_misc.c
  ${ZM_SOURCE_DIR}/zm_sf_erf.c
  ${ZM_SOURCE_DIR}/zm_sf_gamma.c
  ${ZM_SOURCE_DIR}/zm_sf_bessel.c
  ${ZM_SOURCE_DIR}/zm_rand.c
  ${ZM_SOURCE_DIR}/zm_stat.c
  ${ZM_SOURCE_DIR}/zm_complex.c
  ${ZM_SOURCE_DIR}/zm_complex_arith.c
  ${ZM_SOURCE_DIR}/zm_complex_pe.c
  ${ZM_SOURCE_DIR}/zm_raw_vec.c
  ${ZM_SOURCE_DIR}/zm_raw_mat.c
  ${ZM_SOURCE_DIR}/zm_vec.c
  ${ZM_SOURCE_DIR}/zm_vec_array.c
  ${ZM_SOURCE_DIR}/zm_vec_list.c
  ${ZM_SOURCE_DIR}/zm_vec_tree.c
  ${ZM_SOURCE_DIR}/zm_vec_ring.c
  ${ZM_SOURCE_DIR}/zm_mat.c
  ${ZM_SOURCE_DIR}/zm_cvec.c
  ${ZM_SOURCE_DIR}/zm_cmat.c
  ${ZM_SOURCE_DIR}/zm_le.c
  ${ZM_SOURCE_DIR}/zm_le_pivot.c
  ${ZM_SOURCE_DIR}/zm_le_lu.c
  ${ZM_SOURCE_DIR}/zm_le_lq.c
  ${ZM_SOURCE_DIR}/zm_le_minv.c
  ${ZM_SOURCE_DIR}/zm_le_mpinv.c
  ${ZM_SOURCE_DIR}/zm_le_tridiag.c
  ${ZM_SOURCE_DIR}/zm_le_gen.c
  ${ZM_SOURCE_DIR}/zm_le_lyapnov.c
  ${ZM_SOURCE_DIR}/zm_eig.c
  ${ZM_SOURCE_DIR}/zm_mca.c
  ${ZM_SOURCE_DIR}/zm_mca_cluster.c
  ${ZM_SOURCE_DIR}/zm_mca_gmm.c
  ${ZM_SOURCE_DIR}/zm_seq.c
  ${ZM_SOURCE_DIR}/zm_pex.c
  ${ZM_SOURCE_DIR}/zm_pex_eq.c
  ${ZM_SOURCE_DIR}/zm_ip.c
  ${ZM_SOURCE_DIR}/zm_ip_lin.c
  ${ZM_SOURCE_DIR}/zm_ip_lagrange.c
  ${ZM_SOURCE_DIR}/zm_ip_spline.c
  ${ZM_SOURCE_DIR}/zm_ip_akima.c
  ${ZM_SOURCE_DIR}/zm_ip_pex.c
  ${ZM_SOURCE_DIR}/zm_ip_ipio.c
  ${ZM_SOURCE_DIR}/zm_nurbs.c
  ${ZM_SOURCE_DIR}/zm_opt_line.c
  ${ZM_SOURCE_DIR}/zm_opt_lp_stdcnv.c
  ${ZM_SOURCE_DIR}/zm_opt_lp_simplex.c
  ${ZM_SOURCE_DIR}/zm_opt_lp_pdip.c
  ${ZM_SOURCE_DIR}/zm_opt_lcp_lemke.c
  ${ZM_SOURCE_DIR}/zm_opt_lcp_ip.c
  ${ZM_SOURCE_DIR}/zm_opt_qp.c
  ${ZM_SOURCE_DIR}/zm_opt_nm.c
  ${ZM_SOURCE_DIR}/zm_opt_ga.c
  ${ZM_SOURCE_DIR}/zm_opt_dm.c
  ${ZM_SOURCE_DIR}/zm_nle_se.c
  ${ZM_SOURCE_DIR}/zm_nle_dm.c
  ${ZM_SOURCE_DIR}/zm_nle_ss.c
  ${ZM_SOURCE_DIR}/zm_ode.c
  ${ZM_SOURCE_DIR}/zm_ode_dc.c
  ${ZM_SOURCE_DIR}/zm_ode_euler.c
  ${ZM_SOURCE_DIR}/zm_ode_heun.c
  ${ZM_SOURCE_DIR}/zm_ode_rk4.c
  ${ZM_SOURCE_DIR}/zm_ode_rkg.c
  ${ZM_SOURCE_DIR}/zm_ode_erk.c
  ${ZM_SOURCE_DIR}/zm_ode_adams.c
  ${ZM_SOURCE_DIR}/zm_ode_beuler.c
  ${ZM_SOURCE_DIR}/zm_ode_bk4.c
  ${ZM_SOURCE_DIR}/zm_ode_gear.c
  ${ZM_SOURCE_DIR}/zm_ode2.c
  ${ZM_SOURCE_DIR}/zm_fft.c
  ${ZM_SOURCE_DIR}/zm_intg.c
  ${ZM_SOURCE_DIR}/zm_oscil_vdp.c
  ${ZM_SOURCE_DIR}/zm_oscil_kura.c
  ${ZM_SOURCE_DIR}/zm_graph.c
  ${ZM_SOURCE_DIR}/zm_graph_search.c
  ${ZM_SOURCE_DIR}/zm_rrt.c
  ${ZM_SOURCE_DIR}/zm_parse.c
  )
add_library(zm SHARED ${ZM_SOURCE_FILES})
target_include_directories(zm PUBLIC ${ZM_INCLUDE_DIR})
target_link_libraries(zm PUBLIC cure)

# build app
set(ZM_APPS deg2rad rad2deg le psolve pexp seqmod)
foreach(app ${ZM_APPS})
  add_executable(${app} ${ZM_APP_DIR}/${app}.c)
  target_link_libraries(${app} zm)
endforeach()
if(UNIX)
  execute_process(
    COMMAND cure-config -l
    OUTPUT_VARIABLE CURE_LINK
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  set(ZM_LINK "-lzm ${CURE_LINK}")
  add_custom_command(
    OUTPUT zm-config
    COMMAND sh ${CMAKE_BINARY_DIR}/cure-config-gen -I ${CMAKE_INSTALL_PREFIX}/include -L ${CMAKE_INSTALL_PREFIX}/lib -l ${ZM_LINK} -v ${ZM_VERSION} > zm-config
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
  add_custom_target(
    make-zm-config ALL
    DEPENDS make-cure-config-gen
    SOURCES zm-config
    )
endif()

# install
set(PERMISSIONS_755
  OWNER_READ OWNER_WRITE OWNER_EXECUTE
  GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE
  )
install(TARGETS zm
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
install(DIRECTORY ${ZM_INCLUDE_DIR}/ DESTINATION include)
install(TARGETS ${ZM_APPS}
  RUNTIME DESTINATION bin
  )
if(UNIX)
  install(FILES ${CMAKE_BINARY_DIR}/zm-config
    PERMISSIONS ${PERMISSIONS_755}
    DESTINATION bin
    )
endif()
