# ----------------------------------------------------------------------------
#
# Top-level CMake file for holon
#
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.2)

project(holon CXX C)

# holon version
set(HOLON_VERSION_MAJOR 0)
set(HOLON_VERSION_MINOR 1)
set(HOLON_VERSION_PATCH 0)
set(HOLON_VERSION "${HOLON_VERSION_MAJOR}.${HOLON_VERSION_MINOR}.${HOLON_VERSION_PATCH}")

# build third-party modules
add_subdirectory(third_party)
