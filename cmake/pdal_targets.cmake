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
# Add a set of include files to install.
# _component The part of PDAL that the install files belong to.
# _subdir The sub-directory for these include files.
# ARGN The include files.
macro(PDAL_ADD_INCLUDES _subdir)
    install(FILES ${ARGN} DESTINATION ${PDAL_INCLUDE_DIR}/${_subdir})
endmacro(PDAL_ADD_INCLUDES)


###############################################################################
# Add a library target.
# _name The library name.
# _component The part of PDAL that this library belongs to.
# ARGN The source files for the library.
macro(PDAL_ADD_LIBRARY _name)
    add_library(${_name} ${PDAL_LIB_TYPE} ${ARGN})
    
    # must link explicitly against boost.
    target_link_libraries(${_name} ${BOOST_LINKAGE} ${Boost_LIBRARIES})

    install(TARGETS ${_name}
        RUNTIME DESTINATION ${PDAL_BIN_DIR}
        LIBRARY DESTINATION ${PDAL_LIB_DIR}
        ARCHIVE DESTINATION ${PDAL_LIB_DIR})
endmacro(PDAL_ADD_LIBRARY)


###############################################################################
# Add an executable target.
# _name The executable name.
# _component The part of PDAL that this library belongs to.
# ARGN the source files for the library.
macro(PDAL_ADD_EXECUTABLE _name)
    add_executable(${_name} ${ARGN})

    # must link explicitly against boost.
    target_link_libraries(${_name} ${BOOST_LINKAGE} ${Boost_LIBRARIES})
    
    set(PDAL_EXECUTABLES ${PDAL_EXECUTABLES} ${_name})
    install(TARGETS ${_name} RUNTIME DESTINATION ${PDAL_BIN_DIR})
endmacro(PDAL_ADD_EXECUTABLE)


###############################################################################
# Add a plugin target.
# _name The plugin name.
# ARGN the source files for the plugin.
#
# Todo: handle windows/unix variants of the plugin name
# Todo: accept deps for target_link_libraries
macro(PDAL_ADD_PLUGIN _name _type _shortname _srcs _incs)
    add_library(${_name} SHARED ${_srcs} ${_incs})
    target_link_libraries(${_name} ${PDAL_LIB_NAME} ${PCL_LIBRARIES})

    source_group("Header Files\\${_type}\\${_shortname}" FILES ${_incs})
    source_group("Source Files\\${_type}\\${_shortname}" FILES ${_srcs})

    install(TARGETS ${_name}
        RUNTIME DESTINATION ${PDAL_BIN_DIR}
        LIBRARY DESTINATION ${PDAL_LIB_DIR}
        ARCHIVE DESTINATION ${PDAL_LIB_DIR})
endmacro(PDAL_ADD_PLUGIN)
