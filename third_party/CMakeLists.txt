# Prepare "Catch" library for other executables
set(CATCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/catch)
add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR})

if(TESTING_HOLON2)
  # set(Eigen3_DIR "${PROJECT_SOURCE_DIR}/third_party/eigen/cmake/")
  # find_package(Eigen3 3.3 REQUIRED NO_MODULE)
else()
# Build "hayai" library
  add_subdirectory(hayai)

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

endif()
