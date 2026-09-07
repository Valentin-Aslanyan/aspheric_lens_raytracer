cmake_minimum_required(VERSION 3.20)

project(Aspheric_Lens_Raytracer CXX)

include_directories(${CMAKE_SOURCE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

add_library(aspherical aspherical.cpp)
add_library(raytracing raytracing.cpp)

target_link_libraries(raytracing aspherical)

add_executable(Raytrace_GUI Raytrace_GUI.cpp)
add_executable(Raytrace_BMP Raytrace_BMP.cpp)

target_link_libraries(Raytrace_BMP aspherical raytracing)
target_link_libraries(Raytrace_GUI
    aspherical
    raytracing
    SDL2main
    SDL2
)
