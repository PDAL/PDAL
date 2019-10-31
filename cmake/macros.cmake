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
# Add a library target.
# _name The library name.
# ARGN The source files for the library.
#
# The "generate_dimension_hpp" ensures that Dimension.hpp is built before
#  attempting to build anything else in the library.
#
macro(PDAL_ADD_LIBRARY _name)
    add_library(${_name} ${PDAL_LIB_TYPE} ${ARGN})
    add_dependencies(${_name} generate_dimension_hpp)
    set_property(TARGET ${_name} PROPERTY FOLDER "Libraries")
    target_include_directories(${_name} PRIVATE
        ${PDAL_INCLUDE_DIR})
    pdal_target_compile_settings(${_name})
    if (NOT ${_library_type} STREQUAL "STATIC")
        target_compile_definitions(${_name} PRIVATE PDAL_DLL_EXPORT)
    endif()

    install(TARGETS ${_name}
        EXPORT PDALTargets
        RUNTIME DESTINATION ${PDAL_BIN_INSTALL_DIR}
        LIBRARY DESTINATION ${PDAL_LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${PDAL_LIB_INSTALL_DIR})
endmacro(PDAL_ADD_LIBRARY)

###############################################################################
# Add a free library target (one that doesn't depend on PDAL).
# _name The library name.
# _library_type Shared or static
# ARGN The source files for the library.
#
macro(PDAL_ADD_FREE_LIBRARY _name _library_type)
    add_library(${_name} ${_library_type} ${ARGN})
    set_property(TARGET ${_name} PROPERTY FOLDER "Libraries")
    target_include_directories(${_name} PRIVATE
        ${PDAL_INCLUDE_DIR})
    pdal_target_compile_settings(${_name})

    # Don't install static libraries - they're already built into libpdalXXX
    if (NOT ${_library_type} STREQUAL "STATIC")
        target_compile_definitions(${_name} PRIVATE PDAL_DLL_EXPORT)
        install(TARGETS ${_name}
            EXPORT PDALTargets
            RUNTIME DESTINATION ${PDAL_BIN_INSTALL_DIR}
            LIBRARY DESTINATION ${PDAL_LIB_INSTALL_DIR}
            ARCHIVE DESTINATION ${PDAL_LIB_INSTALL_DIR})
    else()
        set_target_properties(${_name} PROPERTIES
            POSITION_INDEPENDENT_CODE TRUE)
    endif()
endmacro(PDAL_ADD_FREE_LIBRARY)

###############################################################################
# Add a plugin target.
# _name The plugin name.
# ARGN :
#    FILES the source files for the plugin
#    LINK_WITH link plugin with libraries
#    INCLUDES header directories
#
# NOTE: _name is the name of a variable that will hold the plugin name
#    when the macro completes
macro(PDAL_ADD_PLUGIN _name _type _shortname)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs FILES LINK_WITH INCLUDES SYSTEM_INCLUDES)
    cmake_parse_arguments(PDAL_ADD_PLUGIN "${options}" "${oneValueArgs}"
        "${multiValueArgs}" ${ARGN})
    if(WIN32)
        set(${_name} "libpdal_plugin_${_type}_${_shortname}")
    else()
        set(${_name} "pdal_plugin_${_type}_${_shortname}")
    endif()

    if (WIN32)
        list(APPEND ${PDAL_ADD_PLUGIN_FILES} ${PDAL_TARGET_OBJECTS})
    endif()

    add_library(${${_name}} SHARED ${PDAL_ADD_PLUGIN_FILES})
    pdal_target_compile_settings(${${_name}})
    target_include_directories(${${_name}} PRIVATE
        ${PROJECT_BINARY_DIR}/include
        ${PDAL_INCLUDE_DIR}
        ${PDAL_ADD_PLUGIN_INCLUDES}
    )
    target_compile_definitions(${${_name}} PRIVATE PDAL_DLL_EXPORT)
    if (PDAL_ADD_PLUGIN_SYSTEM_INCLUDES)
        target_include_directories(${${_name}} SYSTEM PRIVATE
            ${PDAL_ADD_PLUGIN_SYSTEM_INCLUDES})
    endif()
    target_link_libraries(${${_name}}
        PRIVATE
            ${PDAL_BASE_LIB_NAME}
            ${PDAL_UTIL_LIB_NAME}
            ${PDAL_ADD_PLUGIN_LINK_WITH}
            ${WINSOCK_LIBRARY}
    )

    set_property(TARGET ${${_name}} PROPERTY FOLDER "Plugins/${_type}")
    set_target_properties(${${_name}} PROPERTIES
        VERSION "${PDAL_BUILD_VERSION}"
        SOVERSION "${PDAL_API_VERSION}"
        CLEAN_DIRECT_OUTPUT 1)

    install(TARGETS ${${_name}}
        RUNTIME DESTINATION ${PDAL_BIN_INSTALL_DIR}
        LIBRARY DESTINATION ${PDAL_LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${PDAL_LIB_INSTALL_DIR})
    if (APPLE)
        set_target_properties(${${_name}} PROPERTIES
            INSTALL_NAME_DIR "@rpath")
    endif()
endmacro(PDAL_ADD_PLUGIN)

###############################################################################
# Add a test target.
# _name The driver name.
# ARGN :
#    FILES the source files for the test
#    LINK_WITH link test executable with libraries
#    INCLUDES header file directories
#
macro(PDAL_ADD_TEST _name)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs FILES LINK_WITH INCLUDES SYSTEM_INCLUDES)
    cmake_parse_arguments(PDAL_ADD_TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if (WIN32)
        list(APPEND ${PDAL_ADD_TEST_FILES} ${PDAL_TARGET_OBJECTS})
    endif()
    add_executable(${_name} ${PDAL_ADD_TEST_FILES}
        $<TARGET_OBJECTS:${PDAL_TEST_SUPPORT_OBJS}>)
    pdal_target_compile_settings(${_name})
    target_include_directories(${_name} PRIVATE
        ${ROOT_DIR}
        ${PDAL_INCLUDE_DIR}
        ${PDAL_ADD_TEST_INCLUDES}
        ${PROJECT_SOURCE_DIR}/test/unit
        ${PROJECT_BINARY_DIR}/test/unit
        ${PROJECT_BINARY_DIR}/include)
    if (PDAL_ADD_TEST_SYSTEM_INCLUDES)
        target_include_directories(${_name} SYSTEM PRIVATE
            ${PDAL_ADD_TEST_SYSTEM_INCLUDES})
    endif()
    set_property(TARGET ${_name} PROPERTY FOLDER "Tests")
    target_link_libraries(${_name}
        PRIVATE
            ${PDAL_BASE_LIB_NAME}
            ${PDAL_UTIL_LIB_NAME}
            gtest
            ${PDAL_ADD_TEST_LINK_WITH}
            ${WINSOCK_LIBRARY}
    )
    add_test(NAME ${_name}
        COMMAND
            "${PROJECT_BINARY_DIR}/bin/${_name}"
        WORKING_DIRECTORY
            "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/..")
    # Ensure plugins are loaded from build dir
    # https://github.com/PDAL/PDAL/issues/840
    if (WIN32)
        set_property(TEST ${_name} PROPERTY ENVIRONMENT
            "PDAL_DRIVER_PATH=${PROJECT_BINARY_DIR}/bin")
    else()
        set_property(TEST ${_name} PROPERTY ENVIRONMENT
            "PDAL_DRIVER_PATH=${PROJECT_BINARY_DIR}/lib")
    endif()
endmacro(PDAL_ADD_TEST)
