cmake_minimum_required(VERSION 3.0.2)

set( PROJECT_NAME arctrajgenscurve )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_library( ${PROJECT_NAME} SHARED
  src/arctrajgenscurve.cpp
)

target_include_directories( ${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:src>
)


target_link_libraries( ${PROJECT_NAME} PUBLIC
  ${TRAXXS_LIB_NAME}
  SCurveProfile
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

install(DIRECTORY src/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${TRAXXS_LIB_NAME}/impl/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
)
