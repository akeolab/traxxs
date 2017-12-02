cmake_minimum_required(VERSION 3.0.2)

set( LIB_NAME path )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library( ${LIB_NAME} 
  src/${LIB_NAME}.cpp
  src/segment.cpp
)

target_include_directories( ${LIB_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries( ${LIB_NAME}
  arctrajgen
)

