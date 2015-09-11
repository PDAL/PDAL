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
set(PDAL_VERSION_STRING "1.0.1" CACHE STRING "PDAL version" FORCE)

DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

set(PDAL_API_VERSION "1")
set(PDAL_BUILD_VERSION "1.0.0")

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
    set(PDAL_UTIL_LIB_NAME pdal_util)
    set(PDAL_PLANG_LIB_NAME pdal_plang)
else()
    set(PDAL_LIB_NAME pdalcpp)
    set(PDAL_BASE_LIB_NAME pdal_base)
    set(PDAL_UTIL_LIB_NAME pdal_util)
    set(PDAL_PLANG_LIB_NAME pdal_plang)
endif()

set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

set(PDAL_OUTPUT_LIB_DIR "${PDAL_BINARY_DIR}/${PDAL_LIB_INSTALL_DIR}")
set(PDAL_OUTPUT_BIN_DIR "${PDAL_BINARY_DIR}/${PDAL_BIN_INSTALL_DIR}")
set(PDAL_PLUGIN_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/${PDAL_LIB_INSTALL_DIR}")
file(MAKE_DIRECTORY "${PDAL_OUTPUT_LIB_DIR}")
file(MAKE_DIRECTORY "${PDAL_OUTPUT_BIN_DIR}")

# per http://www.cmake.org/Wiki/CMake_RPATH_handling
SET(CMAKE_SKIP_BUILD_RPATH FALSE )
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
if (APPLE)
    SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
endif()
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
IF (APPLE)
    SET(MACOSX_RPATH ON)
endif()
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
    SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

# wipe lib/ drectory on clean. It will have plugins that could be out of date
# in the next build
set_directory_properties(PROPERTY ADDITIONAL_MAKE_CLEAN_FILES "${PDAL_OUTPUT_LIB_DIR}/*")

if(WIN32)
  add_definitions("-DPDAL_DLL_EXPORT=1")
  foreach(config ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${config} CONFIG)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG} "${PDAL_OUTPUT_LIB_DIR}")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG} "${PDAL_OUTPUT_BIN_DIR}")
    # ---[ Windows requires DLLs (shared libraries) to be installed in the same directory as executables
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG} "${PDAL_OUTPUT_BIN_DIR}")
  endforeach(config)
else(WIN32)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PDAL_OUTPUT_LIB_DIR}")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PDAL_OUTPUT_BIN_DIR}")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PDAL_OUTPUT_LIB_DIR}")
endif(WIN32)

set(PDAL_HEADERS_DIR "${PROJECT_SOURCE_DIR}/include/pdal")

#------------------------------------------------------------------------------
# misc user options
#------------------------------------------------------------------------------
if(DEFINED ENV{ORACLE_HOME})
    set(DEFINED_ORACLE_HOME TRUE)
else(DEFINED ENV{ORACLE_HOME})
    set(DEFINED_ORACLE_HOME FALSE)
endif(DEFINED ENV{ORACLE_HOME})

# Choose package components
option(WITH_APPS "Choose if PDAL utilities should be built" TRUE)
add_feature_info("PDAL application" WITH_APPS
    "the PDAL command line application")

option(BUILD_PLUGIN_ATTRIBUTE "Choose if Attribute filter is built" FALSE)
add_feature_info("Attribute plugin" BUILD_PLUGIN_ATTRIBUTE
    "apply attributes to a subset of points")

option(BUILD_PLUGIN_CPD "Choose if Coherent Point Drift kernel is built" FALSE)
add_feature_info("CPD plugin" BUILD_PLUGIN_CPD
    "run Coherent Point Drift on two datasets")

option(BUILD_PLUGIN_GEOWAVE "Choose if GeoWave support should be built" FALSE)
add_feature_info("GeoWave plugin" BUILD_PLUGIN_GEOWAVE
    "Read and Write data using GeoWave")

option(BUILD_PLUGIN_GREYHOUND "Choose if Greyhound support should be built" FALSE)
add_feature_info("Greyhound plugin" BUILD_PLUGIN_GREYHOUND
    "read points from a Greyhound server")

option(BUILD_PLUGIN_HEXBIN "Whether or not HexBin filter is built" FALSE)
add_feature_info("Hexbin plugin" BUILD_PLUGIN_HEXBIN
    "determine boundary and density of a point cloud")

option(BUILD_PLUGIN_ICEBRIDGE "Choose if Icebridge support should be built" FALSE)
add_feature_info("Icebridge plugin" BUILD_PLUGIN_ICEBRIDGE
    "read data in the Icebridge format")

option(BUILD_PLUGIN_MATLAB "Choose if Matlab support should be built" FALSE)
add_feature_info("Matlab plugin" BUILD_PLUGIN_MATLAB
    "write data to a .mat file")

option(BUILD_PLUGIN_MRSID "Choose if MrSID/LiDAR support should be built" FALSE)
add_feature_info("MrSID plugin" BUILD_PLUGIN_MRSID
    "read data in the MrSID format")

option(BUILD_PLUGIN_NITF "Choose if NITF support should be built (only install supported is from http://github.com/hobu/nitro)" FALSE)
add_feature_info("NITF plugin" BUILD_PLUGIN_NITF
    "read/write LAS data wrapped in NITF")

option(BUILD_PLUGIN_OCI "Choose if OCI support should be built" ${DEFINED_ORACLE_HOME})
option(BUILD_PLUGIN_P2G "Choose if Points2Grid support should be built" FALSE)
option(BUILD_PLUGIN_PCL "Choose if PCL support should be built" FALSE)
add_feature_info("PCL plugin" BUILD_PLUGIN_PCL
    "provides PCL-based readers, writers, filters, and kernels")

find_package(PostgreSQL QUIET)
option(BUILD_PLUGIN_PGPOINTCLOUD "Choose if PostgreSQL PointCloud support should be built" ${POSTGRESQL_FOUND})
add_feature_info("PostgreSQL PointCloud plugin" BUILD_PLUGIN_PGPOINTCLOUD
    "read/write PostgreSQL PointCloud objects")

option(BUILD_PLUGIN_SQLITE "Choose if SQLite database support should be built" FALSE)
add_feature_info("SQLite plugin" BUILD_PLUGIN_SQLITE
    "read/write SQLite objects")

option(BUILD_PLUGIN_RIVLIB "Choose if RiVLib support should be built" FALSE)
add_feature_info("RiVLib plugin" BUILD_PLUGIN_RIVLIB
    "read data in the RXP format")

option(BUILD_PLUGIN_PYTHON "Choose if Python support should be built" FALSE)
add_feature_info("Python plugin" BUILD_PLUGIN_PYTHON
    "add features that depend on python")

option(WITH_TESTS "Choose if PDAL unit tests should be built" TRUE)
add_feature_info("Unit tests" WITH_TESTS "PDAL unit tests")

# Choose dependent options

include(CMakeDependentOption)

cmake_dependent_option(BUILD_PGPOINTCLOUD_TESTS "Choose if PostgreSQL PointCloud tests should be built" ON "BUILD_PLUGIN_PGPOINTCLOUD; WITH_TESTS" OFF)
cmake_dependent_option(BUILD_SQLITE_TESTS "Choose if SQLite tests should be built" ON "BUILD_PLUGIN_SQLITE; WITH_TESTS" OFF)
cmake_dependent_option(BUILD_OCI_TESTS "Choose if OCI tests should be built" ON "BUILD_PLUGIN_OCI; WITH_TESTS" OFF)
cmake_dependent_option(BUILD_RIVLIB_TESTS "Choose if RiVLib tests should be built" ON "BUILD_PLUGIN_RIVLIB; WITH_TESTS" OFF)
cmake_dependent_option(BUILD_PIPELINE_TESTS "Choose if pipeline tests should be built" OFF "WITH_APPS; WITH_TESTS" OFF)

if(BUILD_PLUGIN_PGPOINTCLOUD OR BUILD_PLUGIN_OCI OR BUILD_PLUGIN_SQLITE)
    include(${PDAL_CMAKE_DIR}/libxml2.cmake)
endif()

#------------------------------------------------------------------------------
# test harness settings
#------------------------------------------------------------------------------

# Name of test suite runner
set(PDAL_UNIT_TEST pdal_test)

# Enable CTest and submissions to PDAL dashboard at CDash
# http://my.cdash.org/index.php?project=PDAL
option(ENABLE_CTEST "Enable CTest to support submissions of results to CDash at http://cdash.org" FALSE)

#------------------------------------------------------------------------------
# General build settings
#------------------------------------------------------------------------------

set(PDAL_BUILD_TYPE ${CMAKE_BUILD_TYPE})

#------------------------------------------------------------------------------
#  Dependencies.
#------------------------------------------------------------------------------

include(${PDAL_CMAKE_DIR}/boost.cmake)
include(${PDAL_CMAKE_DIR}/gdal.cmake)
include(${PDAL_CMAKE_DIR}/geos.cmake)  # Optional
include(${PDAL_CMAKE_DIR}/geotiff.cmake)  # Optional
include(${PDAL_CMAKE_DIR}/lazperf.cmake)  # Optional
include(${PDAL_CMAKE_DIR}/laszip.cmake)  # Optional
include(${PDAL_CMAKE_DIR}/threads.cmake)
include(${PDAL_CMAKE_DIR}/zlib.cmake)

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

include_directories(vendor/eigen-3.1.91)
include_directories(vendor/nanoflann-1.1.8)
include_directories(vendor/rply-1.1.3)

#------------------------------------------------------------------------------
# subdirectory controls
#------------------------------------------------------------------------------

# PDAL_TARGET_OBJECTS is used to collect the driver object libraries
set(PDAL_TARGET_OBJECTS "")

add_subdirectory(src/util)
add_subdirectory(io)
add_subdirectory(filters)
add_subdirectory(kernels)
add_subdirectory(src)
add_subdirectory(tools)
if (BUILD_PLUGIN_PYTHON)
    add_subdirectory(src/plang)
endif()

if(WITH_TESTS)
  # This is fixed in trunk
  # https://code.google.com/p/googletest/source/detail?r=675
  # but for now, we need this fix
  # http://stackoverflow.com/questions/12558327/google-test-in-visual-studio-2012
  if (MSVC AND MSVC_VERSION EQUAL 1700)
    add_definitions(/D _VARIADIC_MAX=10)
  endif()

  add_subdirectory(vendor/gtest-1.7.0)
  enable_testing()
  include_directories(vendor/gtest-1.7.0/include vendor/gtest-1.7.0)

  if(ENABLE_CTEST)
    message(STATUS
      "Enable CTest to support submissions of results to CDash at http://cdash.org")
    cmake_minimum_required(VERSION 2.8.0)
    # Dashboard has been prepared for experiments
    # http://my.cdash.org/index.php?project=PDAL
    include(CTest)
    message(STATUS
      "Enable CTest to support submissions of results to CDash at http://cdash.org - done")
  endif()

  add_subdirectory(test)

  # Define "make check" as alias for "make test"
  ADD_CUSTOM_TARGET(check COMMAND ctest)
else()
  if(ENABLE_CTEST)
    message(WARNING
      "CTest support requested but WITH_TESTS option not specified to build of PDAL unit tests")
  endif()
endif()

add_subdirectory(plugins)

if(WITH_APPS)
  add_subdirectory(apps)
endif()

#------------------------------------------------------------------------------
# CPACK controls
#------------------------------------------------------------------------------

SET(CPACK_PACKAGE_VERSION_MAJOR ${PDAL_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${PDAL_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${PDAL_VERSION_MINOR})
SET(CPACK_PACKAGE_NAME "PDAL")

SET(CPACK_SOURCE_GENERATOR "TBZ2;TGZ")
SET(CPACK_PACKAGE_VENDOR "PDAL Development Team")
SET(CPACK_RESOURCE_FILE_LICENSE    "${PROJECT_SOURCE_DIR}/LICENSE.txt")

set(CPACK_SOURCE_PACKAGE_FILE_NAME
    "${CMAKE_PROJECT_NAME}-${PDAL_VERSION_STRING}-src")

set(CPACK_SOURCE_IGNORE_FILES
"/\\\\.gitattributes;/\\\\.vagrant;/\\\\.DS_Store;/CVS/;/\\\\.git/;\\\\.swp$;~$;\\\\.\\\\#;/\\\\#")

list(APPEND CPACK_SOURCE_IGNORE_FILES "CMakeScripts/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "CMakeCache.txt")
list(APPEND CPACK_SOURCE_IGNORE_FILES ".xcodeproj")
list(APPEND CPACK_SOURCE_IGNORE_FILES "build.make")
list(APPEND CPACK_SOURCE_IGNORE_FILES "_CPack_Packages")
list(APPEND CPACK_SOURCE_IGNORE_FILES "cmake_install.cmake")
list(APPEND CPACK_SOURCE_IGNORE_FILES "Testing")
list(APPEND CPACK_SOURCE_IGNORE_FILES "PDAL.build/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/bin/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/lib/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/build/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "Makefile")
list(APPEND CPACK_SOURCE_IGNORE_FILES "CMakeFiles")
list(APPEND CPACK_SOURCE_IGNORE_FILES "CTestTestfile.cmake")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/test/data/local/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/doc/doxygen/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/doc/build/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/doc/presentations/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/cmake/examples/")

include(CPack)

add_custom_target(dist COMMAND ${CMAKE_MAKE_PROGRAM} package_source)

export(TARGETS ${PDAL_BASE_LIB_NAME} ${PDAL_UTIL_LIB_NAME}
    FILE "${PDAL_BINARY_DIR}/PDALTargets.cmake")
install(EXPORT PDALTargets DESTINATION lib/pdal/cmake)
include(${PDAL_CMAKE_DIR}/config.cmake)

feature_summary(WHAT ALL INCLUDE_QUIET_PACKAGES)

# TODO: move under scripts/bash-completion ?
if (IS_DIRECTORY /etc/bash_completion.d)
    install(FILES "${PROJECT_SOURCE_DIR}/scripts/bash-completion/pdal" DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/bash_completion.d")
endif()
