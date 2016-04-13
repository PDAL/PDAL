#
# top-level CMake configuration file for PDAL
#
# (based originally on the libLAS files copyright Mateusz Loskot)

cmake_minimum_required(VERSION 2.8.11)

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
set(PDAL_VERSION_STRING "1.2.0" CACHE STRING "PDAL version" FORCE)

DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

set(PDAL_API_VERSION "2")
set(PDAL_BUILD_VERSION "3.0.0")

# Name of C++ library

#
# To facilitate one-library linking, we do special things for various platforms.
#
# On Linux we install a linker script that we call libpdalcpp.so and it
# in turn links libpdal_base.so and libpdal_util.so
#
# On OSX we reexport the symbols from libpdal_util.dylib into libpdalcpp.dylib
# See src/CMakeLists.txt for the rest of the magic.
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
set(PDAL_ARBITER_LIB_NAME pdal_arbiter)

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

set(PDAL_HEADERS_DIR "${PROJECT_SOURCE_DIR}/include/pdal")

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
include(${PDAL_CMAKE_DIR}/threads.cmake)
include(${PDAL_CMAKE_DIR}/zlib.cmake)
include(${PDAL_CMAKE_DIR}/test.cmake)
include(${PDAL_CMAKE_DIR}/ctest.cmake)
include(${PDAL_CMAKE_DIR}/curl.cmake)
include(${PDAL_CMAKE_DIR}/json.cmake)
if (BUILD_PLUGIN_PGPOINTCLOUD OR BUILD_PLUGIN_OCI OR BUILD_PLUGIN_SQLITE)
    include(${PDAL_CMAKE_DIR}/libxml2.cmake)
endif()

#------------------------------------------------------------------------------
# generate the pdal_defines.h header
#------------------------------------------------------------------------------

# from http://stackoverflow.com/questions/1435953/how-can-i-pass-git-sha1-to-compiler-as-definition-using-cmake
include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC GIT_SHA1)

configure_file(
  "${PROJECT_SOURCE_DIR}/gitsha.cpp.in"
  "${PROJECT_SOURCE_DIR}/src/gitsha.cpp")

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
PDAL_ADD_INCLUDES("" "" ${pdal_defines_h})
include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")

include_directories(${PDAL_VENDOR_DIR}/arbiter)
include_directories(${PDAL_VENDOR_DIR}/eigen-3.1.91)

if (PDAL_HAVE_JSONCPP)
    include_directories(${JSONCPP_INCLUDE_DIR})
else()
    include_directories(${PDAL_VENDOR_DIR}/jsoncpp-1.6.2/dist)
endif()

include_directories(${PDAL_VENDOR_DIR}/nanoflann-1.1.8)
include_directories(${PDAL_VENDOR_DIR}/rply-1.1.4)



#------------------------------------------------------------------------------
# subdirectory controls
#------------------------------------------------------------------------------

# PDAL_TARGET_OBJECTS is used to collect the driver object libraries
set(PDAL_TARGET_OBJECTS "")

if (WITH_TESTS)
    enable_testing()
endif()

add_subdirectory(plugins)

include_directories(vendor/pdalboost)
if (WITH_TESTS)
    include (${PDAL_CMAKE_DIR}/gtest.cmake)
    add_subdirectory(test)
endif()
add_subdirectory(vendor/pdalboost)
add_subdirectory(vendor/arbiter)
if (NOT PDAL_HAVE_JSONCPP)
    set(PDAL_JSONCPP_LIB_NAME pdal_jsoncpp)
    add_subdirectory(vendor/jsoncpp-1.6.2/dist)
endif()
add_subdirectory(src/util)
add_subdirectory(io)
add_subdirectory(filters)
add_subdirectory(kernels)
add_subdirectory(src)
add_subdirectory(tools)
if (BUILD_PLUGIN_PYTHON)
    add_subdirectory(src/plang)
endif()
if (WITH_APPS)
    add_subdirectory(apps)
endif()

include (${PDAL_CMAKE_DIR}/cpack.cmake)

add_custom_target(dist COMMAND ${CMAKE_MAKE_PROGRAM} package_source)

export(TARGETS ${PDAL_BASE_LIB_NAME}
    ${PDAL_UTIL_LIB_NAME}
    ${PDAL_BOOST_LIB_NAME}
    ${PDAL_ARBITER_LIB_NAME}
    ${PDAL_JSONCPP_LIB_NAME}
    FILE "${PDAL_BINARY_DIR}/PDALTargets.cmake")
install(EXPORT PDALTargets DESTINATION "${PDAL_LIB_INSTALL_DIR}/pdal/cmake")
include(${PDAL_CMAKE_DIR}/config.cmake)

feature_summary(WHAT ALL INCLUDE_QUIET_PACKAGES)
export(PACKAGE PDAL)

# TODO: move under scripts/bash-completion ?
if (WITH_COMPLETION)
    if (IS_DIRECTORY /etc/bash_completion.d)
        install(FILES "${PROJECT_SOURCE_DIR}/scripts/bash-completion/pdal" DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/bash_completion.d")
    endif()
endif()
