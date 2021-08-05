cmake_minimum_required(VERSION 3.15)
project(rayzzz)

set(CMAKE_CXX_STANDARD 17)

file(GLOB MyHeaders ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB MyCSources *.cpp)

set (HEADERS
    ${MyHeaders}
    )

set (SOURCES 
    ${MyCSources}
    )

source_group ("Header Files" FILES ${HEADERS})
source_group ("Source Files" FILES ${SOURCES})

add_executable(rayzzz ${HEADERS} ${SOURCES})
target_include_directories(rayzzz PUBLIC external/stb/ external/tinygltf/tinygltf include/ external/glm/glm)
