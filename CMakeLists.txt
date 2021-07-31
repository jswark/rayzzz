cmake_minimum_required(VERSION 3.15)
project(rayzzz)

set(CMAKE_CXX_STANDARD 17)

file(GLOB MyHeaders *.h)
file(GLOB MyCSources *.cpp)

set (HEADERS
    ${MyHeaders}
    )

set (SOURCES 
    ${MyCSources}
    )

source_group ("Header Files" FILES ${HEADERS})
source_group ("Source Files" FILES ${SOURCES})


add_executable(rayzzz ${HEADERS} ${SOURCES} modelloader/modelloader.cpp)
target_include_directories(rayzzz PUBLIC external/stb/ external/tinygltf/tinygltf external/glm/glm modelloader/)
