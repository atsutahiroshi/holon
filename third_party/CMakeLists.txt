if(UNIX)
  add_custom_target(
    milib_build ALL
    COMMAND ./make.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/linux
  )
  add_custom_target(
    milib_activate ALL
    COMMAND source activate
    DEPENDS milib_build
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin
  )
endif()

#------------------------------------------------------------
# Add testing
#------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Prepare "Catch" library for other executables
set(CATCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Catch2)
add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR})
