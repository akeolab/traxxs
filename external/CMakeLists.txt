cmake_minimum_required(VERSION 3.0.2)

if( USE_TG_SCURVETRAJGEN )
  add_subdirectory( scurve_traj_gen )
endif()

if( USE_TG_SOFTMOTION )
  add_subdirectory( soft_motion )
endif()

