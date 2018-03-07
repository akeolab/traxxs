cmake_minimum_required(VERSION 3.0.2)
project(traxxs VERSION 0.1.0)

set( TRAXXS_LIB_NAME traxxs )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)

option( USE_TG_SOFTMOTION "use softmotion trajectory generator" ON )
option( USE_TG_SCURVETRAJGEN "use scurve_traj_gen trajectory generator" ON )

option( BUILD_SAMPLES "build samples" ON )

add_subdirectory( external )
add_subdirectory( src )
add_subdirectory( impl )

if( BUILD_SAMPLES )
  add_subdirectory( samples )
endif()

file( COPY python DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )

