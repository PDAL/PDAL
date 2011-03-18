###############################################################################
#
# CMake module to search for MrSID/MG4 libraries
#
# On success, the macro sets the following variables:
# MRSID_FOUND        = if the library was found
# MRSID_LIBRARY      = full path to the library
# MRSID_INCLUDE_DIR  = full path to the headers
# MRSID_VERSION      = version of library which was found
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
# Developed with inspiration from Petr Vanek <petr@scribus.info>
# who wrote similar macro for TOra - http://torasql.com/
#
# Module source: http://github.com/mloskot/workshop/tree/master/cmake/
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
###############################################################################

if(MRSID_INCLUDE_DIR AND MRSID_LIBRARY)
  # Already in cache, be silent
  set(MRSID_FIND_QUIETLY TRUE)
endif()

if(NOT MRSID_FIND_QUIETLY)
  message(STATUS "Searching for MrSID LiDAR library ${MrSID_FIND_VERSION}+")
endif()

if(NOT MRSID_ROOT)
  message(FATAL_ERROR "Missing variable MRSID_ROOT with location of MrSID LiDAR sdk")
endif()

find_path(MRSID_INCLUDE_DIR
          lidar/Base.h
          PATHS
            ${MRSID_ROOT}/include
            ${MRSID_ROOT}/Lidar_DSDK/include
          NO_DEFAULT_PATH)

find_library(MRSID_LIBRARY
             NAMES
               lti_lidar_dsdk
               liblti_lidar_dsdk
             HINTS
               ${MRSID_INCLUDE_DIR}/../lib
             NO_DEFAULT_PATH)


# Handle the QUIETLY and REQUIRED arguments and set MRSID_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MrSID DEFAULT_MSG MRSID_LIBRARY MRSID_INCLUDE_DIR)

