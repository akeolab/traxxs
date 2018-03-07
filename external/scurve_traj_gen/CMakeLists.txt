cmake_minimum_required(VERSION 3.0.2)

set( LIB_NAME "SCurveProfile" )

add_library( ${LIB_NAME} scurve_traj_gen/src/SCurveProfile.cpp )

target_include_directories(${LIB_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/scurve_traj_gen/src>
    $<INSTALL_INTERFACE:scurve_traj_gen/src>
)


## https://rix0r.nl/blog/2015/08/13/cmake-guide/
export(TARGETS ${LIB_NAME} FILE ${LIB_NAME}Config.cmake)

    

