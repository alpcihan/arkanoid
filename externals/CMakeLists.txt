add_library(externals)

find_package(OpenCV REQUIRED)
add_subdirectory(glfw)
add_subdirectory(glm)

target_include_directories( externals PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    glad/include
    stb_image/include
)

target_sources( externals PUBLIC
    glad/src/glad.c
    stb_image/src/stb_image.cpp
)

target_link_libraries( externals PUBLIC
    ${OpenCV_LIBS}  
    glfw
    glm
)