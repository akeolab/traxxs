cmake_minimum_required(VERSION 3.0.2)

set( LIB_NAME "softMotion" )

# softmotion lib is not compatible with c++11 and above
# cf. As of C++11, iostream classes are no longer implicitly convertible to void*
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++03")

set( SOURCE_FILES
  soft-motion-libs/src/Sm_Approx.cpp
  soft-motion-libs/src/Sm_Curve.cpp
  soft-motion-libs/src/Sm_Traj.cpp
  soft-motion-libs/src/debug.c
  soft-motion-libs/src/gnuplot_i.cpp
  soft-motion-libs/src/matrix.c
  soft-motion-libs/src/softMotion.c
  soft-motion-libs/src/softMotion7segments.c
  soft-motion-libs/src/time.c
)
set_source_files_properties(${SOURCE_FILES} PROPERTIES LANGUAGE CXX)

add_library( ${LIB_NAME} STATIC ${SOURCE_FILES} )

set_target_properties(${LIB_NAME}  PROPERTIES LINKER_LANGUAGE CXX)

find_package(LibXml2 REQUIRED)
target_include_directories( ${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${LIBXML2_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${LIBXML2_INCLUDE_DIR}>
)


target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/soft-motion-libs/src>
  $<INSTALL_INTERFACE:soft-motion-libs/src>
)

target_link_libraries( ${LIB_NAME}
    ${LIBXML2_LIBRARIES} 
)

## https://rix0r.nl/blog/2015/08/13/cmake-guide/
export(TARGETS ${LIB_NAME} FILE ${LIB_NAME}Config.cmake)

    

