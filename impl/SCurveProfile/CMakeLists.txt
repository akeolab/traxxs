cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_library( arctrajgenscurve
  src/arctrajgenscurve.cpp
)

target_include_directories( arctrajgenscurve PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:src>
)


target_link_libraries( arctrajgenscurve PUBLIC
  traxxs
  SCurveProfile
)
