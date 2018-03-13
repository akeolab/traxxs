cmake_minimum_required(VERSION 3.0.2)
project(traxxs VERSION 0.1.0)

set( TRAXXS_LIB_NAME traxxs )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)

option(BUILD_DOC "Build documentation" ON)
option( USE_TG_SOFTMOTION "use softmotion trajectory generator" ON )
option( USE_TG_SCURVETRAJGEN "use scurve_traj_gen trajectory generator" OFF )

option( BUILD_SAMPLES "build samples" ON )

add_subdirectory( external )
add_subdirectory( src )
add_subdirectory( impl )

if( BUILD_SAMPLES )
  add_subdirectory( samples )
endif()

file( COPY python DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )


if( BUILD_DOC )
  find_package(Doxygen)
  if (DOXYGEN_FOUND)
      # set input and output files
      set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
      set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

      # request to configure the file
      configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
      message("Doxygen build started")

      # note the option ALL which allows to build the docs together with the application
      add_custom_target( doc ALL
          COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
          WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
          COMMENT "Generating API documentation with Doxygen"
          VERBATIM )
  else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
  endif (DOXYGEN_FOUND)
endif( BUILD_DOC )

#
# CMake package
#
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/traxxsConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

set(CONFIG_PACKAGE_LOCATION share/cmake/traxxs)
set(INCLUDE_INSTALL_DIR include/ )

configure_package_config_file(cmake/traxxsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/traxxsConfig.cmake
    INSTALL_DESTINATION ${CONFIG_PACKAGE_LOCATION}
    PATH_VARS INCLUDE_INSTALL_DIR
)

export(EXPORT traxxsTargets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/traxxsTargets.cmake"
    NAMESPACE traxxs::
)

export(PACKAGE traxxs)

install(EXPORT traxxsTargets
    FILE traxxsTargets.cmake
    NAMESPACE traxxs::
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/traxxsConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/traxxsConfigVersion.cmake
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
)

