cmake_minimum_required(VERSION 3.2)
project(zx11 C)

# set locations
set(ZX11_TOP_DIR     ${PROJECT_SOURCE_DIR}/zx11)
set(ZX11_SOURCE_DIR  ${ZX11_TOP_DIR}/src)
set(ZX11_INCLUDE_DIR ${ZX11_TOP_DIR}/include)
set(ZX11_LIBRARY_DIR ${ZX11_TOP_DIR}/lib)
set(ZX11_APP_DIR     ${ZX11_TOP_DIR}/app)

# set options
option(ZX11_USE_WIDGET
  "Determines whether to use original simple widgets in this library. \
Validate ZX11_USE_XPM for it." ON)
option(ZX11_USE_XEXT "Determines whether to use X11 extension." ON)
option(ZX11_USE_XPM "Determines whether to use XPM." ON)
option(ZX11_USE_PNG "Determines whether to use PNG files." ON)
option(ZX11_USE_JPEG "Determines whether to use JPEG file (read only)." ON)
option(ZX11_USE_DIB "Determines whether to use Microsoft BMP files." ON)
option(ZX11_USE_MAG "Determines whether to use MAG files." ON)
option(ZX11_USE_PDT "Determines whether to use PDT files." ON)

# get version number
file(READ ${ZX11_TOP_DIR}/config.org ZX11_CONFIG_CONTENT)
string(REGEX MATCH
  "PREFIX=(.*)[\r\n\t ]PROJNAME=(.*)[\r\n\t ]VERSION=(.*)[\r\n\t ][\r\n\t ]#.*#.*#.*#.*#.*#.*#.*#.*"
  _ ${ZX11_CONFIG_CONTENT})
set(ZX11_PREFIX   ${CMAKE_MATCH_1})
set(ZX11_PROJNAME ${CMAKE_MATCH_2})
set(ZX11_VERSION  ${CMAKE_MATCH_3})
# message(STATUS "prefix:" ${ZX11_PREFIX})
# message(STATUS "projname:" ${ZX11_PROJNAME})
# message(STATUS "version:" ${ZX11_VERSION})

# build library
set(ZX11_SOURCE_FILES
  ${ZX11_SOURCE_DIR}/zxutil.c
  ${ZX11_SOURCE_DIR}/zxfont.c
  ${ZX11_SOURCE_DIR}/zxinput.c
  ${ZX11_SOURCE_DIR}/zxpixel.c
  ${ZX11_SOURCE_DIR}/zximage.c
  ${ZX11_SOURCE_DIR}/zximage_pnm.c
  )
add_library(zx11 SHARED ${ZX11_SOURCE_FILES})
target_include_directories(zx11 SYSTEM PUBLIC ${ZX11_INCLUDE_DIR})
if(UNIX)
  target_compile_options(zx11
    PRIVATE
    -Wall
    $<$<CONFIG:Debug>:-g3 -O0>
    $<$<CONFIG:Release>:-O3 -funroll-loops>
    )
elseif(MSVC)
  target_compile_options(zx11
    PRIVATE
    /W4
    $<$<CONFIG:Debug>:/Od>
    $<$<CONFIG:Release>:/O2>
    )
endif()

# specify build options
set(ZX11_ADDITIONAL_SOURCE_FILES "")
set(ZX11_LINKER_FLAGS "-lzx11") # used for generating zx11-config
if(ZX11_USE_WIDGET)
  list(APPEND ZX11_ADDITIONAL_SOURCE_FILES ${ZX11_SOURCE_DIR}/zxwidget.c)
endif()
if(ZX11_USE_XEXT)
  target_compile_definitions(zx11 PRIVATE __ZX11_USE_XEXT)
  target_link_libraries(zx11 PUBLIC Xext)
  set(ZX11_LINKER_FLAGS "${ZX11_LINKER_FLAGS} -lXext")
endif()
if(ZX11_USE_XPM)
  list(APPEND ZX11_ADDITIONAL_SOURCE_FILES
    ${ZX11_SOURCE_DIR}/zxpm.c
    ${ZX11_SOURCE_DIR}/zxsprite.c
    )
  target_compile_definitions(zx11 PRIVATE __ZX11_USE_XPM)
  target_link_libraries(zx11 PUBLIC Xpm)
  set(ZX11_LINKER_FLAGS "${ZX11_LINKER_FLAGS} -lXpm")
endif()
if(ZX11_USE_PNG)
  list(APPEND ZX11_ADDITIONAL_SOURCE_FILES ${ZX11_SOURCE_DIR}/zximage_png.c)
  target_compile_definitions(zx11 PRIVATE __ZX11_USE_PNG)
  target_link_libraries(zx11 PUBLIC png z)
  set(ZX11_LINKER_FLAGS "${ZX11_LINKER_FLAGS} -lpng -lz")
endif()
if(ZX11_USE_JPEG)
  list(APPEND ZX11_ADDITIONAL_SOURCE_FILES ${ZX11_SOURCE_DIR}/zximage_jpeg.c)
  target_compile_definitions(zx11 PRIVATE __ZX11_USE_JPEG)
  target_link_libraries(zx11 PUBLIC jpeg)
  set(ZX11_LINKER_FLAGS "${ZX11_LINKER_FLAGS} -ljpeg")
endif()
if(ZX11_USE_DIB)
  list(APPEND ZX11_ADDITIONAL_SOURCE_FILES ${ZX11_SOURCE_DIR}/zximage_dib.c)
  target_compile_definitions(zx11 PRIVATE __ZX11_USE_DIB)
endif()
if(ZX11_USE_MAG)
  list(APPEND ZX11_ADDITIONAL_SOURCE_FILES ${ZX11_SOURCE_DIR}/zximage_mag.c)
  target_compile_definitions(zx11 PRIVATE __ZX11_USE_MAG)
endif()
if(ZX11_USE_PDT)
  list(APPEND ZX11_ADDITIONAL_SOURCE_FILES ${ZX11_SOURCE_DIR}/zximage_pdt.c)
  target_compile_definitions(zx11 PRIVATE __ZX11_USE_PDT)
endif()
set(ZX11_LINKER_FLAGS "${ZX11_LINKER_FLAGS} -lX11")
target_sources(zx11 PRIVATE ${ZX11_ADDITIONAL_SOURCE_FILES})
target_link_libraries(zx11 PUBLIC X11 cure)

# build app
set(ZX11_APPS)
foreach(app ${ZX11_APPS})
  add_executable(${app} ${ZX11_APP_DIR}/${app}.c)
  target_link_libraries(${app} zx11)
endforeach()
if(UNIX)
  add_custom_command(
    OUTPUT zx11-config
    COMMAND echo -n "${ZX11_LINKER_FLAGS} " > zx11_link
    COMMAND sh $<TARGET_FILE_DIR:cure>/cure-config -l >> zx11_link
    COMMAND sh $<TARGET_FILE_DIR:cure>/cure-config-gen -I ${CMAKE_INSTALL_PREFIX}/include -L ${CMAKE_INSTALL_PREFIX}/lib -l \"`cat zx11_link`\" -v ${ZX11_VERSION} > zx11-config
    COMMAND rm -f zx11_link
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
  add_custom_target(
    make-zx11-config ALL
    DEPENDS make-cure-config-gen make-cure-config
    SOURCES zx11-config
    )
endif()

# install
set(PERMISSIONS_755
  OWNER_READ OWNER_WRITE OWNER_EXECUTE
  GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE
  )
install(TARGETS zx11
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
install(DIRECTORY ${ZX11_INCLUDE_DIR}/ DESTINATION include)
install(TARGETS ${ZX11_APPS}
  RUNTIME DESTINATION bin
  )
if(UNIX)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/zx11-config
    PERMISSIONS ${PERMISSIONS_755}
    DESTINATION bin
    )
endif()
