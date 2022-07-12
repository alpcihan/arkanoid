# add source files
file(GLOB_RECURSE SOURCE
"ar-arcanoid.cpp"
"pose/*"
"graphics/*"
)

add_executable(${CMAKE_PROJECT_NAME} ${SOURCE})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC 
    "./"
)
target_link_libraries(${CMAKE_PROJECT_NAME} 
    externals
)