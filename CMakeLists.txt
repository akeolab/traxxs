cmake_minimum_required(VERSION 3.0.2)
project(trajgen)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)

add_subdirectory( lib )
add_subdirectory( arctrajgen )
add_subdirectory( path )
add_subdirectory( impl )

add_subdirectory( samples )

