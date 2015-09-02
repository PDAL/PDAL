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
    install(FILES ${ARGN} DESTINATION ${PDAL_INCLUDE_INSTALL_DIR}/${_subdir})
endmacro(PDAL_ADD_INCLUDES)


###############################################################################
# Add a library target.
# _name The library name.
# _component The part of PDAL that this library belongs to.
# ARGN The source files for the library.
macro(PDAL_ADD_LIBRARY _name)
    add_library(${_name} ${PDAL_LIB_TYPE} ${ARGN})
    set_property(TARGET ${_name} PROPERTY FOLDER "Libraries")

    install(TARGETS ${_name}
        EXPORT PDALTargets
        RUNTIME DESTINATION ${PDAL_BIN_INSTALL_DIR}
        LIBRARY DESTINATION ${PDAL_LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${PDAL_LIB_INSTALL_DIR})
    if (APPLE)
        set_target_properties(${_name} PROPERTIES INSTALL_NAME_DIR
            "@executable_path/../lib")
    endif()
endmacro(PDAL_ADD_LIBRARY)

###############################################################################
# Add an executable target.
# _name The executable name.
# _component The part of PDAL that this library belongs to.
# ARGN the source files for the library.
macro(PDAL_ADD_EXECUTABLE _name)
    add_executable(${_name} ${ARGN})

    # must link explicitly against boost.
    target_link_libraries(${_name} ${Boost_LIBRARIES})

    set(PDAL_EXECUTABLES ${PDAL_EXECUTABLES} ${_name})
    install(TARGETS ${_name}
        EXPORT PDALTargets
        RUNTIME DESTINATION ${PDAL_BIN_INSTALL_DIR})
endmacro(PDAL_ADD_EXECUTABLE)


###############################################################################
# Add a plugin target.
# _name The plugin name.
# ARGN :
#    FILES the srouce files for the plugin
#    LINK_WITH link plugin with libraries
macro(PDAL_ADD_PLUGIN _name _type _shortname)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs FILES LINK_WITH)
    cmake_parse_arguments(PDAL_ADD_PLUGIN "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(WIN32)
        set(${_name} "libpdal_plugin_${_type}_${_shortname}")
    else()
        set(${_name} "pdal_plugin_${_type}_${_shortname}")
    endif()

    if (WIN32)
	    list(APPEND ${PDAL_ADD_PLUGIN_FILES} ${PDAL_TARGET_OBJECTS})
    endif()

    add_library(${${_name}} SHARED ${PDAL_ADD_PLUGIN_FILES})
    target_link_libraries(${${_name}} ${PDAL_BASE_LIB_NAME}
        ${PDAL_ADD_PLUGIN_LINK_WITH})

    set_property(TARGET ${${_name}} PROPERTY FOLDER "Plugins/${_type}")

    install(TARGETS ${${_name}}
        RUNTIME DESTINATION ${PDAL_BIN_INSTALL_DIR}
        LIBRARY DESTINATION ${PDAL_LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${PDAL_LIB_INSTALL_DIR})
    if (APPLE)
        set_target_properties(${${_name}} PROPERTIES INSTALL_NAME_DIR "@loader_path/../lib")
    endif()
endmacro(PDAL_ADD_PLUGIN)

###############################################################################
# Add a test target.
# _name The driver name.
# ARGN :
#    FILES the source files for the test
#    LINK_WITH link test executable with libraries
macro(PDAL_ADD_TEST _name)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs FILES LINK_WITH)
    cmake_parse_arguments(PDAL_ADD_TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    include_directories(${PROJECT_SOURCE_DIR}/test/unit)
    include_directories(${PROJECT_BINARY_DIR}/test/unit)
    set(common_srcs
        ${PROJECT_SOURCE_DIR}/test/unit/Support.cpp
        ${PROJECT_SOURCE_DIR}/test/unit/TestConfig.cpp
    )
    if (WIN32)
        list(APPEND ${PDAL_ADD_TEST_FILES} ${PDAL_TARGET_OBJECTS})
        add_definitions("-DPDAL_DLL_EXPORT=1")
    endif()
    add_executable(${_name} ${PDAL_ADD_TEST_FILES} ${common_srcs})
    set_target_properties(${_name} PROPERTIES COMPILE_DEFINITIONS PDAL_DLL_IMPORT)
    set_property(TARGET ${_name} PROPERTY FOLDER "Tests")
    target_link_libraries(${_name} ${PDAL_BASE_LIB_NAME} gtest
        ${PDAL_ADD_TEST_LINK_WITH})
    add_test(NAME ${_name} COMMAND "${PROJECT_BINARY_DIR}/bin/${_name}" WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/..")
    set_property(TEST ${_name} PROPERTY ENVIRONMENT
      # Ensure plugins are loaded from build dir
      # https://github.com/PDAL/PDAL/issues/840
      "PDAL_DRIVER_PATH=${PROJECT_BINARY_DIR}/lib"
    )
endmacro(PDAL_ADD_TEST)

###############################################################################
# Add a driver. Creates object library and adds files to source_group for windows IDE.
# _type The driver type (e.g., driver, filter, kernel).
# _name The driver name.
# _srcs The list of source files to add.
# _incs The list of includes to add.
# _objs The object library name that is created.
macro(PDAL_ADD_DRIVER _type _name _srcs _incs _objs)
    source_group("Header Files\\${_type}\\${_name}" FILES ${_incs})
    source_group("Source Files\\${_type}\\${_name}" FILES ${_srcs})

    set(libname ${_type}_${_name})
    set(${_objs} $<TARGET_OBJECTS:${libname}>)
	if (NOT WIN32)
		add_definitions("-fPIC")
	endif()
    add_library(${libname} OBJECT ${_srcs} ${_incs})
    set_property(TARGET ${libname} PROPERTY FOLDER "Drivers/${_type}")

    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
        DESTINATION "${PDAL_INCLUDE_INSTALL_DIR}"
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )
endmacro(PDAL_ADD_DRIVER)

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
# Pull the component parts out of the version number.
macro(DISSECT_VERSION)
    # Find version components
    string(REGEX REPLACE "^([0-9]+).*" "\\1"
        PDAL_VERSION_MAJOR "${PDAL_VERSION_STRING}")
    string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1"
        PDAL_VERSION_MINOR "${PDAL_VERSION_STRING}")
    string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1"
        PDAL_VERSION_PATCH "${PDAL_VERSION_STRING}")
    string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.[0-9]+(.*)" "\\1"
        PDAL_CANDIDATE_VERSION "${PDAL_VERSION_STRING}")
endmacro(DISSECT_VERSION)

###############################################################################
# Set the destination directories for installing stuff.
# Sets PDAL_LIB_INSTALL_DIR. Install libraries here.
# Sets PDAL_BIN_INSTALL_DIR. Install binaries here.
# Sets PDAL_INCLUDE_INSTALL_DIR. Install include files here, preferably in a
# subdirectory named after the library in question (e.g.
# "registration/blorgle.h")
macro(SET_INSTALL_DIRS)
  string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
  if (NOT DEFINED PDAL_LIB_INSTALL_DIR)
    set(PDAL_LIB_INSTALL_DIR lib)
  endif ()
    set(PDAL_INCLUDE_INSTALL_ROOT "include/")
    set(PDAL_INCLUDE_INSTALL_DIR
        "${PDAL_INCLUDE_INSTALL_ROOT}/${PROJECT_NAME_LOWER}/")
    set(PDAL_DOC_INCLUDE_DIR
        "share/doc/${PROJECT_NAME_LOWER}-${PDAL_VERSION_MAJOR}.${PDAL_VERSION_MINOR}")
    set(PDAL_BIN_INSTALL_DIR "bin")
    set(PDAL_PLUGIN_INSTALL_DIR "share/pdal/plugins")
    if(WIN32)
        set(PDALCONFIG_INSTALL_DIR "cmake")
    else(WIN32)
        set(PDALCONFIG_INSTALL_DIR
            "share/${PROJECT_NAME_LOWER}-${PDAL_VERSION_MAJOR}.${PDAL_VERSION_MINOR}")
    endif(WIN32)
endmacro(SET_INSTALL_DIRS)
