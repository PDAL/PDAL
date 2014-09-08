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

# Options for building PDAL.

# Build shared libraries by default.
option(PDAL_BUILD_STATIC "Build PDAL as a static library" OFF)
if(PDAL_BUILD_STATIC)
  set(PDAL_LIB_PREFIX ${CMAKE_STATIC_LIBRARY_PREFIX})
  set(PDAL_LIB_SUFFIX ${CMAKE_STATIC_LIBRARY_SUFFIX})
  set(PDAL_LIB_TYPE "STATIC")
  set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX})
else(PDAL_BUILD_STATIC)
  set(PDAL_LIB_PREFIX ${CMAKE_SHARED_LIBRARY_PREFIX})
  set(PDAL_LIB_SUFFIX ${CMAKE_SHARED_LIBRARY_SUFFIX})
  set(PDAL_LIB_TYPE "SHARED")
#  set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_SHARED_LIBRARY_SUFFIX})
  if(WIN32)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_IMPORT_LIBRARY_SUFFIX})
  endif(WIN32)
endif(PDAL_BUILD_STATIC)
mark_as_advanced(PDAL_BUILD_STATIC)

if (CMAKE_VERSION VERSION_GREATER 2.8.10)
    if (CMAKE_VERSION VERSION_GREATER 2.8.12)
        set(PDAL_LINKAGE "PUBLIC;general")
    else()
        set(PDAL_LINKAGE "LINK_PUBLIC;general")
    endif()

    set(BOOST_LINKAGE "LINK_PUBLIC;general")

else()
    set(PDAL_LINKAGE "")
    set(BOOST_LINKAGE "")
endif()

