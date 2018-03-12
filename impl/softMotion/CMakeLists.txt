cmake_minimum_required(VERSION 3.0.2)

set( PROJECT_NAME traxxs_softmotion )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_library( ${PROJECT_NAME} SHARED
  src/traxxs_softmotion.cpp
)

target_include_directories( ${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


target_link_libraries( ${PROJECT_NAME} PUBLIC
  ${TRAXXS_LIB_NAME}
  PRIVATE softMotion
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib EXPORT traxxsTargets)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
)
