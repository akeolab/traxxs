cmake_minimum_required(VERSION 3.0.2)
set( LIB_NAME arctrajgen )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_library( ${LIB_NAME} 
  src/${LIB_NAME}.cpp
)

target_include_directories( ${LIB_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

