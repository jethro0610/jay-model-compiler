cmake_minimum_required(VERSION 3.10)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(JayModelCompiler)

file(GLOB SOURCES
    main.cpp
)

include_directories("Libraries/tinygltf")
include_directories("Libraries/glm")
include_directories("Libraries/vector_const")
add_executable(JayModelCompiler ${SOURCES})
