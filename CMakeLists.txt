cmake_minimum_required(VERSION 3.0.2)
project(traxxs)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)

add_subdirectory( lib )
add_subdirectory( src )
add_subdirectory( impl )

add_subdirectory( samples )

file( COPY python DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
