cmake_minimum_required(VERSION 3.0.2)

if( USE_TG_SCURVETRAJGEN )
  add_subdirectory( SCurveProfile )
endif()

if( USE_TG_SOFTMOTION )
  add_subdirectory( softMotion )
endif()
