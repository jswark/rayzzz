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

set(MODELLIB_SOURCES
        modelloader/modelloader.h
        modelloader/modelloader.cpp)
set(MODELLIB_NAME modelloader)
add_library(${MODELLIB_NAME} OBJECT ${MODELLIB_SOURCES})
target_include_directories(${MODELLIB_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(${MODELLIB_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/modelloader)
target_include_directories(${MODELLIB_NAME} PUBLIC external/tinygltf/tinygltf external/glm/glm)


add_executable(rayzzz ${HEADERS} ${SOURCES} ${MODELLIB_NAME})
target_include_directories(rayzzz PUBLIC external/stb/ external/tinygltf/tinygltf include/)
