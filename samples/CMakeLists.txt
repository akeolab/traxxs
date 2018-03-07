cmake_minimum_required(VERSION 3.0.2)
project(trajgen_samples)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

file( COPY python DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )

if( USE_TG_SCURVETRAJGEN )
  add_executable( sample_lib_scurveprofile sample_lib_scurveprofile.cpp )
  target_link_libraries(sample_lib_scurveprofile SCurveProfile)

  add_executable( sample_arctrajgen_scurve sample_arctrajgen_scurve.cpp )
  target_link_libraries(sample_arctrajgen_scurve arctrajgenscurve)
endif()

if( USE_TG_SOFTMOTION )
  add_executable( sample_arctrajgen_softmotion sample_arctrajgen_softmotion.cpp )
  target_link_libraries(sample_arctrajgen_softmotion arctrajgensoftmotion)
endif()

add_executable( sample_path sample_path.cpp )
target_link_libraries(sample_path traxxs)

add_executable( sample_path_stack sample_path_stack.cpp )
target_link_libraries(sample_path_stack traxxs)

add_executable( sample_path_cart sample_path_cart.cpp )
target_link_libraries(sample_path_cart traxxs)

if( USE_TG_SOFTMOTION )
  add_executable( sample_traj sample_traj.cpp )
  target_link_libraries(sample_traj traxxs arctrajgensoftmotion)

  add_executable( sample_traj_cart sample_traj_cart.cpp )
  target_link_libraries(sample_traj_cart traxxs arctrajgensoftmotion)

  add_executable( sample_tracker_time_cart sample_tracker_time_cart.cpp )
  target_link_libraries(sample_tracker_time_cart traxxs arctrajgensoftmotion)

  add_executable( sample_tracker_space_cart sample_tracker_space_cart.cpp )
  target_link_libraries(sample_tracker_space_cart traxxs arctrajgensoftmotion)
endif()
