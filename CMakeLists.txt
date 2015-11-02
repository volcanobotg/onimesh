cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(OniMesh)

# *******************************************
# OpenNI2

FIND_LIBRARY( OPENNI2_LIBRARY
             NAMES OpenNI2
             HINTS ${PC_OPENNI2_LIBDIR} ${PC_OPENNI2_LIBRARY_DIRS} /usr/lib
             PATHS "$ENV{PROGRAMFILES}/OpenNI2/Lib${OPENNI2_SUFFIX}"    "$ENV{PROGRAMW6432}/OpenNI2/Lib${OPENNI2_SUFFIX}" "$ENV{PROGRAMW6432}/OpenNI2"
             PATH_SUFFIXES lib lib64
)
FIND_PATH( OPENNI2_INCLUDE_DIR OpenNI.h
          HINTS ${PC_OPENNI2_INCLUDEDIR} ${PC_OPENNI2_INCLUDE_DIRS} 
                  /usr/include/openni2 /usr/include/ni2
                  PATHS "$ENV{PROGRAMFILES}/OpenNI2/include" "$ENV{PROGRAMW6432}/OpenNI2/include"
          PATH_SUFFIXES openni2 ni2)
 
# End OpenNI2
# ******************************************************************

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OPENNI2_LIBRARY})
link_directories(${PCL_LIBRARY_DIRS} ${OPENNI2_LIBRARY})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
file(GLOB OniMesh_SRC
    "src/*.h"
    "src/*.cpp"
)
# *************************************************************************************
# Set the output directories
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BUILD_DIR}/bin ) # this is for the binary file
set (CMAKE_LIBRARIES_OUTPUT_DIRECTORY ${BUILD_DIR}/bin) # this is for the libraries
# *************************************************************************************

add_executable(OniMesh ${OniMesh_SRC} ${OPENNI2_LIBRARY})
 
target_link_libraries (OniMesh ${PCL_LIBRARIES} ${OPENNI2_LIBRARY})
