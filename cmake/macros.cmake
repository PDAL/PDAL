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
    pdal_lib_compile_settings(${_name})
    if (PDAL_LIB_TYPE STREQUAL "SHARED")
        target_compile_definitions(${_name} PRIVATE PDAL_DLL_EXPORT)
    else ()
        target_compile_definitions(${_name} PUBLIC PDAL_STATIC_BUILD)
        set_target_properties(${_name} PROPERTIES POSITION_INDEPENDENT_CODE ON)
    endif()

    target_compile_features (${_name}
      INTERFACE
        # Enable C++17 standard compliance
        cxx_std_17
    )

    set_property (TARGET ${_name}
      PROPERTY
        # Enable C++17 standard compliance
        CXX_STANDARD 17
    )

    install(TARGETS ${_name}
        EXPORT PDALTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endmacro(PDAL_ADD_LIBRARY)

###############################################################################
# Deprecated compatibility stub for external consumers that still check for
# this macro.
macro(PDAL_ADD_FREE_LIBRARY _name _library_type _pdal_lib_type)
    message(DEPRECATION
        "PDAL_ADD_FREE_LIBRARY is deprecated and no longer creates a target. "
        "Use standard CMake target commands instead.")
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
    if(MSVC)
        set(${_name} "libpdal_plugin_${_type}_${_shortname}")
    else()
        set(${_name} "pdal_plugin_${_type}_${_shortname}")
    endif()

    if (WIN32)
        list(APPEND ${PDAL_ADD_PLUGIN_FILES} ${PDAL_TARGET_OBJECTS})
    endif()

    add_library(${${_name}} ${PDAL_LIB_TYPE} ${PDAL_ADD_PLUGIN_FILES})
    set_property(GLOBAL APPEND PROPERTY PDAL_PLUGIN_TARGETS ${${_name}})
    if ("${_type}" STREQUAL "kernel")
        set_property(GLOBAL APPEND PROPERTY PDAL_KERNEL_PLUGIN_TARGETS
            ${${_name}})
    endif()
    pdal_target_compile_settings(${${_name}})
    target_include_directories(${${_name}} PRIVATE
        ${PROJECT_BINARY_DIR}/include
        ${PDAL_INCLUDE_DIR}
        ${PDAL_ADD_PLUGIN_INCLUDES}
    )
    set_property (TARGET ${${_name}}
      PROPERTY
        # Enable C++17 standard compliance
        CXX_STANDARD 17
    )
    if (PDAL_LIB_TYPE STREQUAL "SHARED")
        target_compile_definitions(${${_name}} PRIVATE PDAL_DLL_EXPORT)
    else()
        target_compile_definitions(${${_name}} PRIVATE PDAL_STATIC_BUILD)
        set_target_properties(${${_name}} PROPERTIES POSITION_INDEPENDENT_CODE ON)
        set(_pdal_static_plugin_symbols)
        foreach(_pdal_plugin_file IN LISTS PDAL_ADD_PLUGIN_FILES)
            if (IS_ABSOLUTE "${_pdal_plugin_file}")
                set(_pdal_plugin_path "${_pdal_plugin_file}")
            else()
                set(_pdal_plugin_path
                    "${CMAKE_CURRENT_SOURCE_DIR}/${_pdal_plugin_file}")
            endif()
            if (EXISTS "${_pdal_plugin_path}")
                file(READ "${_pdal_plugin_path}" _pdal_plugin_contents)
                string(REGEX MATCHALL
                    "CREATE_SHARED_(STAGE|KERNEL)\\([ \t\r\n]*[A-Za-z_][A-Za-z0-9_]*"
                    _pdal_static_stage_matches
                    "${_pdal_plugin_contents}")
                foreach(_pdal_static_stage_match IN LISTS
                    _pdal_static_stage_matches)
                    string(REGEX REPLACE
                        ".*\\([ \t\r\n]*([A-Za-z_][A-Za-z0-9_]*).*"
                        "pdal_static_plugin_registration_\\1"
                        _pdal_static_plugin_symbol
                        "${_pdal_static_stage_match}")
                    list(APPEND _pdal_static_plugin_symbols
                        ${_pdal_static_plugin_symbol})
                endforeach()
                string(REGEX MATCHALL
                    "CREATE_SHARED_PLUGIN\\([^,]+,[^,]+,[ \t\r\n]*[A-Za-z_][A-Za-z0-9_]*"
                    _pdal_static_plugin_matches
                    "${_pdal_plugin_contents}")
                foreach(_pdal_static_plugin_match IN LISTS
                    _pdal_static_plugin_matches)
                    string(REGEX REPLACE
                        ".*,[ \t\r\n]*([A-Za-z_][A-Za-z0-9_]*).*"
                        "pdal_static_plugin_registration_\\1"
                        _pdal_static_plugin_symbol
                        "${_pdal_static_plugin_match}")
                    list(APPEND _pdal_static_plugin_symbols
                        ${_pdal_static_plugin_symbol})
                endforeach()
            endif()
        endforeach()
        if (_pdal_static_plugin_symbols)
            list(REMOVE_DUPLICATES _pdal_static_plugin_symbols)
            set_target_properties(${${_name}} PROPERTIES
                PDAL_STATIC_PLUGIN_REGISTRATION_SYMBOLS
                    "${_pdal_static_plugin_symbols}")
        endif()
    endif()
    if (PDAL_ADD_PLUGIN_SYSTEM_INCLUDES)
        target_include_directories(${${_name}} SYSTEM PRIVATE
            ${PDAL_ADD_PLUGIN_SYSTEM_INCLUDES})
    endif()
    set(_pdal_plugin_link_libraries ${PDAL_ADD_PLUGIN_LINK_WITH}
        ${WINSOCK_LIBRARY})
    list(REMOVE_ITEM _pdal_plugin_link_libraries
        ${PDAL_LIB_NAME}
        PDAL::PDAL
        ${PDAL_LIBRARIES})
    set_target_properties(${${_name}} PROPERTIES
        PDAL_STATIC_PLUGIN_LINK_LIBRARIES
            "${_pdal_plugin_link_libraries}")
    target_link_libraries(${${_name}}
        PRIVATE
            ${PDAL_LIB_NAME}
            ${PDAL_ADD_PLUGIN_LINK_WITH}
            ${WINSOCK_LIBRARY}
    )
    foreach(_include IN LISTS PDAL_ADD_PLUGIN_INCLUDES)
        if(TARGET ${_include})
            target_include_directories(${${_name}} PRIVATE $<TARGET_PROPERTY:${_include},INTERFACE_INCLUDE_DIRECTORIES>)
        else()
            target_include_directories(${${_name}} PRIVATE "${include}")
        endif()
    endforeach()

    set_property(TARGET ${${_name}} PROPERTY FOLDER "Plugins/${_type}")
    set_target_properties(${${_name}} PROPERTIES
        VERSION "${PDAL_BUILD_VERSION}"
        SOVERSION "${PDAL_API_VERSION}"
        CLEAN_DIRECT_OUTPUT 1)

    install(TARGETS ${${_name}}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endmacro(PDAL_ADD_PLUGIN)

###############################################################################
# Link the PDAL library into an executable.
#
# Static stage/kernel registration happens through static initializers in each
# stage translation unit. Static archives only pull objects that satisfy a
# referenced symbol, so static PDAL consumers must whole-archive pdalcpp to make
# every built-in registration initializer available at runtime.
function(PDAL_TARGET_LINK_PDAL _target _scope)
    if (PDAL_LIB_TYPE STREQUAL "STATIC")
        if (CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
            target_link_libraries(${_target}
                ${_scope}
                    "$<LINK_LIBRARY:WHOLE_ARCHIVE,${PDAL_LIB_NAME}>")
        elseif(MSVC)
            target_link_libraries(${_target}
                ${_scope}
                    ${PDAL_LIB_NAME})
            target_link_options(${_target}
                ${_scope}
                    "LINKER:/WHOLEARCHIVE:$<TARGET_FILE:${PDAL_LIB_NAME}>")
        elseif(APPLE)
            target_link_libraries(${_target}
                ${_scope}
                    ${PDAL_LIB_NAME})
            target_link_options(${_target}
                ${_scope}
                    "LINKER:-force_load,$<TARGET_FILE:${PDAL_LIB_NAME}>")
        else()
            target_link_libraries(${_target}
                ${_scope}
                    "-Wl,--whole-archive"
                    ${PDAL_LIB_NAME}
                    "-Wl,--no-whole-archive")
        endif()
    else()
        target_link_libraries(${_target}
            ${_scope}
                ${PDAL_LIB_NAME})
    endif()
endfunction()

###############################################################################
# Link libraries into a target. Static plugin registration anchors are kept
# undefined until link time so only the registration objects are pulled from
# static plugin archives.
function(PDAL_TARGET_LINK_STATIC_PLUGINS _target _scope)
    foreach(_library IN LISTS ARGN)
        set(_registration_symbols)
        if (PDAL_LIB_TYPE STREQUAL "STATIC" AND TARGET ${_library})
            add_dependencies(${_target} ${_library})
            get_target_property(_registration_symbols ${_library}
                PDAL_STATIC_PLUGIN_REGISTRATION_SYMBOLS)
            if (NOT _registration_symbols)
                set(_registration_symbols)
            endif()
            foreach(_registration_symbol IN LISTS _registration_symbols)
                if (MSVC)
                    target_link_options(${_target}
                        ${_scope}
                            "LINKER:/INCLUDE:${_registration_symbol}")
                elseif(APPLE)
                    target_link_options(${_target}
                        ${_scope}
                            "LINKER:-u,_${_registration_symbol}")
                else()
                    target_link_options(${_target}
                        ${_scope}
                            "LINKER:-u,${_registration_symbol}")
                endif()
            endforeach()

            target_link_libraries(${_target}
                ${_scope}
                    "$<TARGET_FILE:${_library}>")

            get_target_property(_plugin_link_libraries ${_library}
                PDAL_STATIC_PLUGIN_LINK_LIBRARIES)
            if (_plugin_link_libraries)
                target_link_libraries(${_target}
                    ${_scope}
                        ${_plugin_link_libraries})
            endif()
        else()
            target_link_libraries(${_target}
                ${_scope}
                    ${_library})
        endif()
    endforeach()
endfunction()

###############################################################################
# Add a test target.
# _name The driver name.
# ARGN :
#    FILES the source files for the test
#    LINK_WITH link test executable with libraries
#    INCLUDES header file directories
#

if(NOT TARGET GTest::gtest)
    include (${PDAL_CMAKE_DIR}/gtest.cmake)
endif()

macro(PDAL_ADD_TEST _name)

    if (NOT WITH_TESTS)
        return()
    endif(NOT WITH_TESTS)

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
    if(NOT USE_EXTERNAL_GTEST AND NOT MSVC)
        target_compile_options(${_name} BEFORE PRIVATE
            "-iquote${ROOT_DIR}/vendor/gtest/include")
    endif()
    if (PDAL_ADD_TEST_SYSTEM_INCLUDES)
        target_include_directories(${_name} SYSTEM PRIVATE
            ${PDAL_ADD_TEST_SYSTEM_INCLUDES})
    endif()
    set_property(TARGET ${_name} PROPERTY FOLDER "Tests")
    target_link_libraries(${_name}
        PRIVATE
            GTest::gtest
            ${WINSOCK_LIBRARY}
    )
    PDAL_TARGET_LINK_STATIC_PLUGINS(${_name} PRIVATE ${PDAL_ADD_TEST_LINK_WITH})
    PDAL_TARGET_LINK_PDAL(${_name} PRIVATE)
    foreach(_include IN LISTS PDAL_ADD_TEST_INCLUDES)
        if(TARGET ${_include})
            target_include_directories(${_name} PRIVATE $<TARGET_PROPERTY:${_include},INTERFACE_INCLUDE_DIRECTORIES>)
        else()
            target_include_directories(${_name} PRIVATE "${include}")
        endif()
    endforeach()

    add_test(NAME ${_name}
        COMMAND
            "${PROJECT_BINARY_DIR}/bin/${_name}"
        WORKING_DIRECTORY
            "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/..")
    # Ensure plugins are loaded from build dir
    # https://github.com/PDAL/PDAL/issues/840
    set(_pdal_test_environment)
    if (WIN32)
        list(APPEND _pdal_test_environment
            "PDAL_DRIVER_PATH=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
    else()
        list(APPEND _pdal_test_environment
            "PDAL_DRIVER_PATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
        if (APPLE AND PDAL_LIB_TYPE STREQUAL "SHARED")
            list(APPEND _pdal_test_environment
                "DYLD_LIBRARY_PATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:$ENV{DYLD_LIBRARY_PATH}")
        endif()
    endif()
    set_property(TEST ${_name} PROPERTY ENVIRONMENT
        ${_pdal_test_environment})
endmacro(PDAL_ADD_TEST)
