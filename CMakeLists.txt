cmake_minimum_required(VERSION 3.21)
project(ar-arcanoid)

set (CMAKE_CXX_STANDARD 20)
# option(BUILD_opencv_gapi "BUILD_opencv_gapi" OFF)
# option(BUILD_opencv_test_gapi "BUILD_opencv_test_gapi" OFF)

# move resources 
file(GLOB RESOURCES
  "resources/*"
)
file(COPY ${RESOURCES} DESTINATION ./src)

include_directories(${CMAKE_BINARY_DIR})

add_subdirectory(externals)
add_subdirectory(src)