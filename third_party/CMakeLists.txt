# Prepare "Catch" library for other executables
set(CATCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/catch)
add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR})

# Build "hayai" library
if(NOT TESTING_HOLON2)
  add_subdirectory(hayai)
endif()

add_subdirectory(cure)
add_subdirectory(zm)
add_subdirectory(zeo)
add_subdirectory(roki)
add_subdirectory(dzco)
if(UNIX)
  add_subdirectory(liw)
  add_subdirectory(zx11)
  add_subdirectory(roki-gl)
endif()
