cmake_minimum_required(VERSION 3.0.2)
project(trajgen_samples)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

file( COPY python DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )

add_executable( sample_lib_scurveprofile sample_lib_scurveprofile.cpp )
target_link_libraries(sample_lib_scurveprofile SCurveProfile)

add_executable( sample_arctrajgen_scurve sample_arctrajgen_scurve.cpp )
target_link_libraries(sample_arctrajgen_scurve arctrajgenscurve)

add_executable( sample_arctrajgen_softmotion sample_arctrajgen_softmotion.cpp )
target_link_libraries(sample_arctrajgen_softmotion arctrajgensoftmotion)

add_executable( sample_path sample_path.cpp )
target_link_libraries(sample_path traxxs)

add_executable( sample_path_stack sample_path_stack.cpp )
target_link_libraries(sample_path_stack traxxs)

add_executable( sample_path_cart sample_path_cart.cpp )
target_link_libraries(sample_path_cart traxxs)
