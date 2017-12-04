cmake_minimum_required(VERSION 3.0.2)
set( LIB_NAME traxxs )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


file(GLOB SRC_FILES
    "**/*.cpp"
)

add_library( ${LIB_NAME} 
  ${SRC_FILES}
)

target_include_directories( ${LIB_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include>
)

