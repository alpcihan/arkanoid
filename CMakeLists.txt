cmake_minimum_required(VERSION 3.21)
project(ar-arcanoid)

set (CMAKE_CXX_STANDARD 20)
configure_file(./src/config/directories.h.in config/directories.h)
# option(BUILD_opencv_gapi "BUILD_opencv_gapi" OFF)
# option(BUILD_opencv_test_gapi "BUILD_opencv_test_gapi" OFF)

include_directories(${CMAKE_BINARY_DIR})

add_subdirectory(externals)
add_subdirectory(src)