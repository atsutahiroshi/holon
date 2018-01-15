if(UNIX)
  add_custom_target(
    milib_build ALL
    COMMAND ./make.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/linux
  )
  add_custom_target(
    milib_activate ALL
    COMMAND bash -c "source activate"
    DEPENDS milib_build
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin
  )
endif()
