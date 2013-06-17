##########################################################################
# These macros were taken from the Point Cloud Library (pointclouds.org) #
# and have been modified for PDAL. License details follow.               #
##########################################################################
# Software License Agreement (BSD License)                               #
#                                                                        #
# Point Cloud Library (PCL) - www.pointclouds.org                        #
# Copyright (c) 2009-2012, Willow Garage, Inc.                           #
# Copyright (c) 2012-, Open Perception, Inc.                             #
# Copyright (c) XXX, respective authors.                                 #
#                                                                        #
# All rights reserved.                                                   #
#                                                                        #
# Redistribution and use in source and binary forms, with or without     #
# modification, are permitted provided that the following conditions     #
# are met:                                                               #
#                                                                        #
#  * Redistributions of source code must retain the above copyright      #
#    notice, this list of conditions and the following disclaimer.       #
#  * Redistributions in binary form must reproduce the above             #
#    copyright notice, this list of conditions and the following         #
#    disclaimer in the documentation and/or other materials provided     #
#    with the distribution.                                              #
#  * Neither the name of the copyright holder(s) nor the names of its    #
#    contributors may be used to endorse or promote products derived     #
#    from this software without specific prior written permission.       #
#                                                                        #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    #
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      #
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS      #
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE         #
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,    #
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,   #
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;       #
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER       #
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT     #
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN      #
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE        #
# POSSIBILITY OF SUCH DAMAGE.                                            #
##########################################################################

###############################################################################
# Pull the component parts out of the version number.
macro(DISSECT_VERSION)
    # Find version components
    string(REGEX REPLACE "^([0-9]+).*" "\\1"
        PDAL_VERSION_MAJOR "${PDAL_VERSION_STRING}")
    string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1"
        PDAL_VERSION_MINOR "${PDAL_VERSION_STRING}")
    string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+)" "\\1"
        PDAL_VERSION_PATCH "${PDAL_VERSION_STRING}")
    string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.[0-9]+(.*)" "\\1"
        PDAL_CANDIDATE_VERSION "${PDAL_VERSION_STRING}")
endmacro(DISSECT_VERSION)

###############################################################################
# Get the operating system information. Generally, CMake does a good job of
# this. Sometimes, though, it doesn't give enough information. This macro will
# distinguish between the UNIX variants. Otherwise, use the CMake variables
# such as WIN32 and APPLE and CYGWIN.
# Sets OS_IS_64BIT if the operating system is 64-bit.
# Sets LINUX if the operating system is Linux.
macro(GET_OS_INFO)
    string(REGEX MATCH "Linux" OS_IS_LINUX ${CMAKE_SYSTEM_NAME})
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(OS_IS_64BIT TRUE)
    else(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(OS_IS_64BIT FALSE)
    endif(CMAKE_SIZEOF_VOID_P EQUAL 8)
endmacro(GET_OS_INFO)

###############################################################################
# Set the destination directories for installing stuff.
# Sets LIB_INSTALL_DIR. Install libraries here.
# Sets BIN_INSTALL_DIR. Install binaries here.
# Sets INCLUDE_INSTALL_DIR. Install include files here, preferably in a
# subdirectory named after the library in question (e.g.
# "registration/blorgle.h")
macro(SET_INSTALL_DIRS)
  if (NOT DEFINED LIB_INSTALL_DIR)
    set(PDAL_LIB_DIR "lib")
  endif (NOT DEFINED LIB_INSTALL_DIR)
    set(PDAL_INCLUDE_ROOT
        "include/")
    set(PDAL_INCLUDE_DIR "${PDAL_INCLUDE_ROOT}/${PROJECT_NAME_LOWER}/")
    set(PDAL_DOC_DIR "share/doc/${PROJECT_NAME_LOWER}-${PDAL_VERSION_MAJOR}.${PDAL_VERSION_MINOR}")
    set(PDAL_BIN_DIR "bin")
    set(PDAL_PKGCFG_DIR "${PDAL_LIB_DIR}/pkgconfig")
    if(WIN32)
        set(PDALCONFIG_INSTALL_DIR "cmake")
    else(WIN32)
        set(PDALCONFIG_INSTALL_DIR "share/${PROJECT_NAME_LOWER}-${PDAL_VERSION_MAJOR}.${PDAL_VERSION_MINOR}")
    endif(WIN32)
endmacro(SET_INSTALL_DIRS)

###############################################################################
# Add a set of include files to install.
# _component The part of PDAL that the install files belong to.
# _subdir The sub-directory for these include files.
# ARGN The include files.
macro(PDAL_ADD_INCLUDES _component _subdir)
    install(FILES ${ARGN} DESTINATION ${PDAL_INCLUDE_DIR}/${_subdir}
        COMPONENT ${_component})
endmacro(PDAL_ADD_INCLUDES)
