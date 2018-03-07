cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


file(GLOB SRC_FILES
    "**/*.cpp"
)

add_library( ${TRAXXS_LIB_NAME} SHARED
  ${SRC_FILES}
)

target_include_directories( ${TRAXXS_LIB_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS ${TRAXXS_LIB_NAME} DESTINATION lib EXPORT traxxsTargets)

install(DIRECTORY ../include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN "*.in" EXCLUDE
)


