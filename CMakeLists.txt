#
# top-level CMake configuration file for PDAL
#
# (based originally on the libLAS files copyright Mateusz Loskot)

cmake_minimum_required(VERSION 2.8.12)

project(PDAL CXX C)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

set(ROOT_DIR "${PROJECT_SOURCE_DIR}")
include(${ROOT_DIR}/cmake/common.cmake NO_POLICY_SCOPE)

#------------------------------------------------------------------------------
# internal cmake settings
#------------------------------------------------------------------------------

set(CMAKE_COLOR_MAKEFILE ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
include(FeatureSummary)

# Allow advanced users to generate Makefiles printing detailed commands
mark_as_advanced(CMAKE_VERBOSE_MAKEFILE)

#------------------------------------------------------------------------------
# PDAL general settings
#------------------------------------------------------------------------------

# the next line is the ONLY place in the entire pdal system where
# the version info is hard-coded
set(PDAL_VERSION_STRING "1.4.0" CACHE STRING "PDAL version" FORCE)

DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

set(PDAL_API_VERSION "4")
set(PDAL_BUILD_VERSION "5.0.0")

# Name of C++ library

#
# To facilitate one-library linking, we do special things for various platforms.
#
# On Linux we install a linker script that we call libpdalcpp.so and it
# in turn links libpdal_base.so and libpdal_util.so
#
# On OSX we reexport the symbols from libpdal_util.dylib into libpdalcpp.dylib
# See below for the rest of the magic.
#
if (APPLE OR WIN32)
    set(PDAL_LIB_NAME pdalcpp)
    set(PDAL_BASE_LIB_NAME pdalcpp)
else()
    set(PDAL_LIB_NAME pdalcpp)
    set(PDAL_BASE_LIB_NAME pdal_base)
endif()
set(PDAL_UTIL_LIB_NAME pdal_util)
set(PDAL_PLANG_LIB_NAME pdal_plang)
set(PDAL_BOOST_LIB_NAME pdal_boost)
#set(PDAL_ARBITER_LIB_NAME pdal_arbiter)

set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

set(PDAL_OUTPUT_LIB_DIR "${PDAL_BINARY_DIR}/${PDAL_LIB_INSTALL_DIR}")
set(PDAL_OUTPUT_BIN_DIR "${PDAL_BINARY_DIR}/${PDAL_BIN_INSTALL_DIR}")
set(PDAL_PLUGIN_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/${PDAL_LIB_INSTALL_DIR}")
file(MAKE_DIRECTORY "${PDAL_OUTPUT_LIB_DIR}")
file(MAKE_DIRECTORY "${PDAL_OUTPUT_BIN_DIR}")

include(${PDAL_CMAKE_DIR}/rpath.cmake)

# wipe lib/ drectory on clean. It will have plugins that could be out of date
# in the next build
set_directory_properties(PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${PDAL_OUTPUT_LIB_DIR}/*")

if(WIN32)
  add_definitions("-DPDAL_DLL_EXPORT=1")
  foreach(config ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${config} CONFIG)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG} "${PDAL_OUTPUT_LIB_DIR}")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG} "${PDAL_OUTPUT_BIN_DIR}")
    # ---[ Windows requires DLLs (shared libraries) to be installed in
    # ---[ the same directory as executables
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG} "${PDAL_OUTPUT_BIN_DIR}")
  endforeach(config)
else(WIN32)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PDAL_OUTPUT_LIB_DIR}")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PDAL_OUTPUT_BIN_DIR}")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PDAL_OUTPUT_LIB_DIR}")
endif(WIN32)

# Choose package components

include(${PDAL_CMAKE_DIR}/options.cmake)

#------------------------------------------------------------------------------
# General build settings
#------------------------------------------------------------------------------

set(PDAL_BUILD_TYPE ${CMAKE_BUILD_TYPE})

#------------------------------------------------------------------------------
#  Dependencies.
#------------------------------------------------------------------------------

include(${PDAL_CMAKE_DIR}/gdal.cmake)
include(${PDAL_CMAKE_DIR}/geos.cmake)
include(${PDAL_CMAKE_DIR}/geotiff.cmake)  # Optional (not really)
include(${PDAL_CMAKE_DIR}/lazperf.cmake)  # Optional
include(${PDAL_CMAKE_DIR}/laszip.cmake)  # Optional
include(${PDAL_CMAKE_DIR}/pdaljni.cmake)  # Optional
include(${PDAL_CMAKE_DIR}/threads.cmake)
include(${PDAL_CMAKE_DIR}/zlib.cmake)
include(${PDAL_CMAKE_DIR}/test.cmake)
include(${PDAL_CMAKE_DIR}/ctest.cmake)
include(${PDAL_CMAKE_DIR}/curl.cmake)
if (CURL_FOUND)
    set(PDAL_ARBITER_LIB_NAME pdal_arbiter)
endif()
include(${PDAL_CMAKE_DIR}/rply.cmake)
include(${PDAL_CMAKE_DIR}/json.cmake)
include(${PDAL_CMAKE_DIR}/libxml2.cmake)
include(${PDAL_CMAKE_DIR}/dimension.cmake)

#------------------------------------------------------------------------------
# generate the pdal_defines.h header
#------------------------------------------------------------------------------

# from http://stackoverflow.com/questions/1435953/how-can-i-pass-git-sha1-to-compiler-as-definition-using-cmake
include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC GIT_SHA1)

configure_file(
  "${PROJECT_SOURCE_DIR}/gitsha.cpp.in"
  "${PROJECT_SOURCE_DIR}/pdal/gitsha.cpp")

# needs to come before configuration of pdal_defines
if(APPLE)
    option(PDAL_BUNDLE "Create PDAL as Application Bundle on OSX" FALSE)
    if (PDAL_BUNDLE)
        set(PDAL_APP_BUNDLE 1)
    endif()
endif()

set(pdal_defines_h_in "${CMAKE_CURRENT_SOURCE_DIR}/pdal_defines.h.in")
set(pdal_defines_h "${CMAKE_CURRENT_BINARY_DIR}/include/pdal/pdal_defines.h")
configure_file(${pdal_defines_h_in} ${pdal_defines_h})

#------------------------------------------------------------------------------
# subdirectory controls
#------------------------------------------------------------------------------

# PDAL_TARGET_OBJECTS is used to collect the driver object libraries
set(PDAL_TARGET_OBJECTS "")

if (WITH_TESTS)
    enable_testing()
endif()

add_subdirectory(plugins)

#include_directories(vendor/pdalboost)
if (WITH_TESTS)
    include (${PDAL_CMAKE_DIR}/gtest.cmake)
    add_subdirectory(test)
endif()
add_subdirectory(dimbuilder)
add_subdirectory(vendor/pdalboost)
add_subdirectory(vendor/arbiter)

if (NOT PDAL_HAVE_JSONCPP)
    add_subdirectory(vendor/jsoncpp/dist)
endif()
add_subdirectory(pdal/util)
add_subdirectory(tools)
if (BUILD_PLUGIN_PYTHON)
    set(PYTHON_VERSION_STRING "something" CACHE STRING "Python version" FORCE)
    add_subdirectory(pdal/plang)
endif()
add_subdirectory(apps)

#
# On OSX we reexport the symbols in libpdal_util.dylib into libpdalcpp.dylib
# so that users only need link libpdalcpp.
#
if (APPLE)
    set(PDAL_REEXPORT "-Wl,-reexport_library,$<TARGET_FILE:${PDAL_UTIL_LIB_NAME}>")
    #
    # This allows the rpath reference for the reexported library (above) to
    # be found.
    #
    set(PDAL_LIBDIR "-L$<TARGET_FILE_DIR:${PDAL_UTIL_LIB_NAME}>")
endif()

file(GLOB BASE_SRCS
    ${PDAL_FILTERS_DIR}/*.cpp
    ${PDAL_IO_DIR}/*.cpp
    ${PDAL_KERNELS_DIR}/*.cpp
    ${PDAL_SRC_DIR}/*.cpp)
file(GLOB_RECURSE PRIVATE_SRCS
    ${PDAL_FILTERS_DIR}/private/*.cpp
    ${PDAL_IO_DIR}/private/*.cpp
    ${PDAL_KERNELS_DIR}/private/*.cpp
    ${PDAL_SRC_DIR}/private/*.cpp)
list(APPEND SRCS ${BASE_SRCS} ${PRIVATE_SRCS})
if (NOT PDAL_HAVE_LIBXML2)
    file(GLOB XML_SRCS
        io/Ilvis2MetadataReader.cpp
        io/Ilvis2Metadata.cpp
        pdal/DbWriter.cpp
        pdal/DbReader.cpp
        pdal/XMLSchema.cpp)
    list(REMOVE_ITEM SRCS ${XML_SRCS})
endif()
PDAL_ADD_LIBRARY(${PDAL_BASE_LIB_NAME} ${SRCS} ${RPLY_SRCS})

#
# Interface include directories allow downstream project to get the directory
# without specification.
#
target_include_directories(${PDAL_BASE_LIB_NAME}
    PRIVATE
        ${ROOT_DIR}
        ${PROJECT_BINARY_DIR}/include
        ${PDAL_VENDOR_DIR}
        ${PDAL_VENDOR_DIR}/eigen
        ${PDAL_VENDOR_DIR}/pdalboost
        ${PDAL_JSONCPP_INCLUDE_DIR}
        ${LIBXML2_INCLUDE_DIR}
    INTERFACE
        ${GDAL_INCLUDE_DIR}
)
target_link_libraries(${PDAL_BASE_LIB_NAME}
    PUBLIC
        ${CMAKE_THREAD_LIBS_INIT}
        ${GDAL_LIBRARY}
        ${GEOS_LIBRARY}
        ${GEOTIFF_LIBRARY}
        ${LASZIP_LIBRARY}
        ${LIBXML2_LIBRARIES}
        ${ZLIB_LIBRARIES}
        ${CURL_LIBRARIES}
        ${WINSOCK_LIBRARY}
    PRIVATE
        ${PDAL_REEXPORT}
        ${PDAL_UTIL_LIB_NAME}
        ${PDAL_ARBITER_LIB_NAME}
    ${JSON_CPP_LINK_TYPE}
        ${PDAL_JSONCPP_LIB_NAME}
    INTERFACE
        ${PDAL_LIBDIR}
)
target_compile_definitions(${PDAL_BASE_LIB_NAME}
    PRIVATE
        ${LASZIP_DEFINES}
)
set_target_properties(${PDAL_BASE_LIB_NAME} PROPERTIES
    VERSION ${PDAL_BUILD_VERSION}
    SOVERSION ${PDAL_API_VERSION}
    CLEAN_DIRECT_OUTPUT 1)
#
# On Linux, we install a linker script as libpdalcpp.so.  That file
# specifies linking in libpdal_base.so and libpdal_util.so.  This allows
# users to link a single library, libpdalcpp
#
if (UNIX AND NOT APPLE)
    set(LIBNAME ${CMAKE_SHARED_LIBRARY_PREFIX}${PDAL_LIB_NAME})
    install(FILES ${LIBNAME} DESTINATION ${PDAL_LIB_INSTALL_DIR}
        RENAME ${LIBNAME}${CMAKE_SHARED_LIBRARY_SUFFIX})
endif()

#
# Installation
#
install(DIRECTORY ${PDAL_INCLUDE_DIR}/pdal
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN "gitsha.h"
    PATTERN "pdal/private" EXCLUDE
)
install(DIRECTORY ${PDAL_KERNELS_DIR}
    DESTINATION include/pdal
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN "private" EXCLUDE
)
install(DIRECTORY ${PDAL_IO_DIR}
    DESTINATION include/pdal
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN "private" EXCLUDE
)
install(DIRECTORY ${PDAL_FILTERS_DIR}
    DESTINATION include/pdal
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN "private" EXCLUDE
)

install(FILES ${DIMENSION_OUTFILE} ${pdal_defines_h}
  DESTINATION include/pdal
)

#
# CPACK
#
include (${PDAL_CMAKE_DIR}/cpack.cmake)

add_custom_target(dist COMMAND ${CMAKE_MAKE_PROGRAM} package_source)

export(
    TARGETS
        ${PDAL_BASE_LIB_NAME} ${PDAL_UTIL_LIB_NAME}
    FILE
        "${PDAL_BINARY_DIR}/PDALTargets.cmake")

install(
    EXPORT
        PDALTargets
    DESTINATION
        "${PDAL_LIB_INSTALL_DIR}/pdal/cmake")
include(${PDAL_CMAKE_DIR}/config.cmake)
feature_summary(WHAT ALL INCLUDE_QUIET_PACKAGES)
export(PACKAGE PDAL)

# TODO: move under scripts/bash-completion ?
if (WITH_COMPLETION)
    if (IS_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/bash-completion/completions)
        install(
            FILES
                "${PROJECT_SOURCE_DIR}/scripts/bash-completion/pdal"
            DESTINATION
                "${CMAKE_INSTALL_PREFIX}/share/bash-completion/completions")
    elseif (IS_DIRECTORY /etc/bash_completion.d)
        install(
            FILES
                "${PROJECT_SOURCE_DIR}/scripts/bash-completion/pdal"
            DESTINATION
                "${CMAKE_INSTALL_PREFIX}/etc/bash_completion.d")
    endif()
endif()
