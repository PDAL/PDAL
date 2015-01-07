#
# top-level CMake configuration file for PDAL
#
# (based originally on the libLAS files copyright Mateusz Loskot)

cmake_minimum_required(VERSION 2.8.6)

project(PDAL CXX C)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

#------------------------------------------------------------------------------
# internal cmake settings
#------------------------------------------------------------------------------

set(CMAKE_COLOR_MAKEFILE ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
include(FeatureSummary)

# Allow advanced users to generate Makefiles printing detailed commands
mark_as_advanced(CMAKE_VERBOSE_MAKEFILE)

# Path to additional CMake modules
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules" ${CMAKE_MODULE_PATH})

if (CMAKE_MAJOR_VERSION GREATER 2)
cmake_policy(SET CMP0022 OLD) # interface link libraries
cmake_policy(SET CMP0042 OLD) # osx rpath
endif()

# per http://www.cmake.org/Wiki/CMake_RPATH_handling
SET(CMAKE_SKIP_BUILD_RPATH FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
    SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")


# Provided modules
include(CMakeDependentOption)


#------------------------------------------------------------------------------
# PDAL general settings
#------------------------------------------------------------------------------

include("${PDAL_SOURCE_DIR}/cmake/pdal_options.cmake")
include("${PDAL_SOURCE_DIR}/cmake/pdal_targets.cmake")
include("${PDAL_SOURCE_DIR}/cmake/pdal_utils.cmake")

# the next line is the ONLY place in the entire pdal system where
# the version info is hard-coded
set(PDAL_VERSION_STRING "1.0.0.b1" CACHE STRING "PDAL version" FORCE)

DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

SET(PDAL_LIB_SOVERSION "0.0.0")

# Name of C++ library
set(PDAL_LIB_NAME pdalcpp)

set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

set(PDAL_OUTPUT_LIB_DIR "${PDAL_BINARY_DIR}/${PDAL_LIB_DIR}")
set(PDAL_OUTPUT_BIN_DIR "${PDAL_BINARY_DIR}/${PDAL_BIN_DIR}")
file(MAKE_DIRECTORY "${PDAL_OUTPUT_LIB_DIR}")
file(MAKE_DIRECTORY "${PDAL_OUTPUT_BIN_DIR}")
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

option(BUILD_PLUGIN_GREYHOUND "Choose if Greyhound support should be built" FALSE)
add_feature_info("Greyhound plugin" BUILD_PLUGIN_GREYHOUND
    "read points from a Greyhound server")

option(BUILD_PLUGIN_HEXBIN "Whether or not HexBin filter is built" FALSE)
add_feature_info("Hexbin plugin" BUILD_PLUGIN_HEXBIN
    "determine boundary and density of a point cloud")

option(BUILD_PLUGIN_ICEBRIDGE "Choose if Icebridge support should be built" FALSE)
add_feature_info("Icebridge plugin" BUILD_PLUGIN_ICEBRIDGE
    "read data in the Icebridge format")

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

option(BUILD_PLUGIN_PGPOINTCLOUD "Choose if PostgreSQL PointCloud support should be built" TRUE)
add_feature_info("PostgreSQL PointCloud plugin" BUILD_PLUGIN_PGPOINTCLOUD
    "read/write PostgreSQL PointCloud objects")

option(BUILD_PLUGIN_SQLITE "Choose if SQLite database support should be built" FALSE)
add_feature_info("SQLite plugin" BUILD_PLUGIN_SQLITE
    "read/write SQLite objects")

option(BUILD_PLUGIN_RIVLIB "Choose if RiVLib support should be built" FALSE)
add_feature_info("RiVLib plugin" BUILD_PLUGIN_RIVLIB
    "read data in the RXP format")

# Choose to use pkg-config or not
option(WITH_PKGCONFIG "Choose whether a pkgconfig file (PDAL.pc) should be installed" TRUE)

option(WITH_TESTS "Choose if PDAL unit tests should be built" TRUE)
add_feature_info("Unit tests" WITH_TESTS "PDAL unit tests")

# Choose dependent options
cmake_dependent_option(BUILD_PGPOINTCLOUD_TESTS "Choose if PostgreSQL PointCloud tests should be built" ON "BUILD_PLUGIN_PGPOINTCLOUD; WITH_TESTS" OFF)
cmake_dependent_option(BUILD_SQLITE_TESTS "Choose if SQLite tests should be built" ON "BUILD_PLUGIN_SQLITE; WITH_TESTS" OFF)
cmake_dependent_option(BUILD_OCI_TESTS "Choose if OCI tests should be built" ON "BUILD_PLUGIN_OCI; WITH_TESTS" OFF)
cmake_dependent_option(BUILD_RIVLIB_TESTS "Choose if RiVLib tests should be built" ON "BUILD_PLUGIN_RIVLIB; WITH_TESTS" OFF)
cmake_dependent_option(BUILD_PIPELINE_TESTS "Choose if pipeline tests should be built" OFF "WITH_APPS; WITH_TESTS" OFF)
cmake_dependent_option(BUILD_PLUGIN_PYTHON "Choose if Python support should be built" ON "WITH_PYTHON" OFF)


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

if(WIN32)
  add_definitions(-DWIN32_LEAN_AND_MEAN)
  if(MSVC)
    option(PDAL_USE_STATIC_RUNTIME "Use the static runtime" FALSE)

    if(PDAL_USE_STATIC_RUNTIME)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MT")

      # Note that the CMake cache will still show /MD
      # http://www.cmake.org/Wiki/CMake_FAQ#Dynamic_Replace
      foreach(flag_var
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
        if(${flag_var} MATCHES "/MD")
          string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
        endif(${flag_var} MATCHES "/MD")
      endforeach(flag_var)
    endif(PDAL_USE_STATIC_RUNTIME)
  endif(MSVC)
endif(WIN32)

# note we default to debug mode
if(NOT MSVC_IDE)
  if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel" FORCE)
  endif()
  message(STATUS "Setting PDAL build type - ${CMAKE_BUILD_TYPE}")
endif()

set(PDAL_BUILD_TYPE ${CMAKE_BUILD_TYPE})

#------------------------------------------------------------------------------
# Platform and compiler specific settings
#------------------------------------------------------------------------------

if(WIN32)
  if (MSVC)
    set(PDAL_COMPILER_MSVC 1)
    if (MSVC12)
      set(PDAL_COMPILER_VC12 1)
    elseif (MSVC11)
      set(PDAL_COMPILER_VC11 1)
    elseif (MSVC10)
      set(PDAL_COMPILER_VC10 1)
    elseif (MSVC9)
      set(PDAL_COMPILER_VC9 1)
    elseif (MSVC8)
      set(PDAL_COMPILER_VC8 1)
    endif()

    # check for MSVC 8+
    if (NOT (MSVC_VERSION VERSION_LESS 1400))
      add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
      add_definitions(-D_CRT_SECURE_NO_WARNINGS)
      add_definitions(-D_CRT_NONSTDC_NO_WARNING)
      add_definitions(-D_SCL_SECURE_NO_WARNINGS)
      add_definitions(-DNOMINMAX)
      add_definitions(-DBOOST_ALL_DYN_LINK)
    endif()

    if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
      string(REGEX REPLACE "/W[0-4]" "/W4"
        CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    endif()

    # check for MSVC 9+
    if (NOT (MSVC_VERSION VERSION_LESS 1500))
      if (NOT ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION} LESS 2.8.6)
        include(ProcessorCount)
        ProcessorCount(N)
        if(NOT N EQUAL 0)
          set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   /MP${N}")
  	      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP${N}")
        endif()
      endif()
    endif()

  endif(MSVC)

  set(CMAKE_INCLUDE_PATH "c:/OSGeo4W64/include;$ENV{CMAKE_INCLUDE_PATH}")
  set(CMAKE_LIBRARY_PATH "c:/OSGeo4W64/lib;$ENV{CMAKE_LIBRARY_PATH}")
  set(CMAKE_PREFIX_PATH "c:/OSGeo4W64/cmake;$ENV{CMAKE_LIBRARY_PATH}")

  set(PDAL_PLATFORM_WIN32 1)
else()
  # not WIN32

  # Recommended C++ compilation flags
  # -Weffc++
  set(PDAL_COMMON_CXX_FLAGS
    "-Wextra -Wall -Wno-unused-parameter -Wno-unused-variable -Wpointer-arith -Wcast-align -Wcast-qual -Wfloat-equal -Wredundant-decls -Wno-long-long")

    if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PDAL_COMMON_CXX_FLAGS}")
        if (CMAKE_COMPILER_IS_GNUCXX)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem /usr/local/include")
        endif()
        set(PDAL_COMPILER_GCC 1)
    elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" OR
            "${CMAKE_CXX_COMPILER}" MATCHES "clang")
         set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PDAL_COMMON_CXX_FLAGS} -Qunused-arguments")
         set(PDAL_COMPILER_CLANG 1)
         set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
         set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem /usr/local/include")
    endif()
endif(WIN32)

if (APPLE)
    set(SO_EXT dylib)
    set(CMAKE_FIND_FRAMEWORK "LAST")
elseif(WIN32)
    set(SO_EXT dll)
else()
    set(SO_EXT so)
endif(APPLE)

#------------------------------------------------------------------------------
# settings for Boost dependencies
#------------------------------------------------------------------------------

find_package(Threads)
set_package_properties(Threads PROPERTIES DESCRIPTION "The thread library of the system"
    TYPE REQUIRED)

# Default to using static, multithreaded libraries for
# linking under MSVC.  This is because we show users how to
# use boostpro.com installer and install those options when linking
# on windows in the compilation documentation.
if(WIN32)
  if (MSVC)
    set(Boost_USE_STATIC_LIBS   OFF)
    set(Boost_USE_MULTITHREADED ON)

    if (PDAL_USE_STATIC_RUNTIME)
      set(Boost_USE_STATIC_RUNTIME ON)
    endif(PDAL_USE_STATIC_RUNTIME)
  endif(MSVC)
endif(WIN32)

find_package(Boost QUIET 1.53 COMPONENTS program_options iostreams filesystem system)
set_package_properties(Boost PROPERTIES TYPE REQUIRED
    PURPOSE "Provides convenience, e.g., program_options, filesytem")

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})

  # make these available for the user to set.
  mark_as_advanced(CLEAR Boost_INCLUDE_DIR)
  mark_as_advanced(CLEAR Boost_LIBRARY_DIRS)
  link_directories(${Boost_LIBRARY_DIRS})

endif(Boost_FOUND)

#------------------------------------------------------------------------------
# settings for GDAL & related dependencies
#------------------------------------------------------------------------------

#
# GDAL/OGR support (required)
#
find_package(GDAL QUIET 1.9.0)
set_package_properties(GDAL PROPERTIES TYPE REQUIRED
    PURPOSE "Provides general purpose raster, vector, and reference system support")
if (GDAL_FOUND)
    include_directories("${GDAL_INCLUDE_DIR}")
    mark_as_advanced(CLEAR GDAL_INCLUDE_DIR)
    mark_as_advanced(CLEAR GDAL_LIBRARY)
else()
    message(FATAL_ERROR "GDAL support is required")
endif()


#
# GEOS (optional)
#
find_package(GEOS QUIET 3.3)
set_package_properties(GEOS PROPERTIES TYPE OPTIONAL
    PURPOSE "Provides general purpose geometry support")
if (GEOS_FOUND)
    find_file(GEOS_VERSION_H version.h "${GEOS_INCLUDE_DIR}/geos")
    if ("${GEOS_VERSION_H}" STREQUAL "GEOS_VERSION_H-NOTFOUND")
        set(GEOS_LIBRARY "")
        set(GEOS_FOUND FALSE)
    else()
        include_directories("${GEOS_INCLUDE_DIR}")
        set(PDAL_HAVE_GEOS 1)
    endif()
else()
    set(GEOS_LIBRARY "")
endif()


#
# GeoTIFF support (optional)
#
option(WITH_GEOTIFF "Choose if GeoTIFF support should be built" TRUE)
if(WITH_GEOTIFF)
    find_package(GeoTIFF QUIET 1.3.0)
    set_package_properties(GeoTIFF PROPERTIES TYPE OPTIONAL)
    if(GEOTIFF_FOUND)
        include_directories("${GEOTIFF_INCLUDE_DIR}")
        set(PDAL_HAVE_LIBGEOTIFF 1)
        mark_as_advanced(CLEAR TIFF_INCLUDE_DIR)
        mark_as_advanced(CLEAR TIFF_LIBRARY)
        if (WIN32)
            set(TIFF_NAMES libtiff_i)
        endif(WIN32)
    else()
        set(WITH_GEOTIFF FALSE)
        set(GEOTIFF_LIBRARY "")
    endif()
endif()


#
# ICONV (optional)
#
# GDAL and libxml2 require iconv.  The library is supplied by OSGeo4W.
#
option(WITH_ICONV "Choose if IConv support should be built" TRUE)
if(WITH_ICONV)
    find_package(ICONV)
    set_package_properties(ICONV PROPERTIES TYPE OPTIONAL)
    if (ICONV_FOUND)
        mark_as_advanced(CLEAR ICONV_INCLUDE_DIR)
        mark_as_advanced(CLEAR ICONV_LIBRARIES)
        include_directories(${ICONV_INCLUDE_DIR})
        set(PDAL_HAVE_ICONV 1)
    endif()
endif()


#
# LASZIP support (optional)
#
option(WITH_LASZIP "Choose if LASzip support should be built" TRUE)
option(WITH_STATIC_LASZIP "Choose if LASzip should be statically linked" FALSE)
mark_as_advanced(WITH_STATIC_LASZIP)

if(WITH_LASZIP)
    find_package(LASzip QUIET 1.0.1)
    set_package_properties(LASzip PROPERTIES TYPE RECOMMENDED
        PURPOSE "Provides LASzip compression")
    if(LASZIP_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES "${LASZIP_LIBRARY}")
        include_directories(${LASZIP_INCLUDE_DIR})
        mark_as_advanced(CLEAR LASZIP_INCLUDE_DIR)
        mark_as_advanced(CLEAR LASZIP_LIBRARY)
        mark_as_advanced(CLEAR LASZIP_VERSION)
        set(PDAL_HAVE_LASZIP 1)
    else()
        set(LASZIP_LIBRARY "")
        set(WITH_LASZIP FALSE)
    endif()
endif()

option(WITH_LAZPERF "Choose to use laz-perf compression for database drivers" FALSE)
if (WITH_LAZPERF)
    find_package(Lazperf)
    set_package_properties(Lazperf PROPERTIES TYPE OPTIONAL)
    if (LAZPERF_FOUND)
        include_directories(${LAZPERF_INCLUDE_DIR})
        set(PDAL_HAVE_LAZPERF 1)
    else()
        set(WITH_LAZPERF FALSE)
    endif()
endif()

#
# ZLIB support - required
#
find_package(ZLIB)
set_package_properties(ZLIB PROPERTIES TYPE REQUIRED
    PURPOSE "Compression support in BPF")
if(ZLIB_FOUND)
    set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES} "${ZLIB_LIBRARY}")
	include_directories(${ZLIB_INCLUDE_DIR})
	mark_as_advanced(CLEAR ZLIB_INCLUDE_DIR)
	mark_as_advanced(CLEAR ZLIB_LIBRARY)
	set(PDAL_HAVE_ZLIB 1)
endif()


#
# Python - optional
#
option(WITH_PYTHON "Choose if Python support (for PLang filters) should be built" TRUE)
if(WITH_PYTHON)
	find_package(PythonInterp QUIET)
    set_package_properties(PythonInterp PROPERTIES TYPE OPTIONAL)
    find_package(PythonLibs QUIET 2.4)
    set_package_properties(PythonLibs PROPERTIES TYPE OPTIONAL)
    if(PYTHONLIBS_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES "${PYTHON_LIBRARY}")
        include_directories(SYSTEM ${PYTHON_INCLUDE_DIR})
        add_definitions(-DHAVE_PYTHON=1)
        set(PDAL_HAVE_PYTHON 1)

	find_package(NumPy QUIET 1.5 REQUIRED)
        include_directories(SYSTEM ${NUMPY_INCLUDE_DIR})
        message(STATUS "Found Python: ${PYTHON_LIBRARY}")
    else()
        set(WITH_PYTHON FALSE)
        set(PYTHON_LIBRARY "")
    endif()
endif()


#
# libxml2 (used iff OCI or PGPOINTCLOUD or SQLITE)
#
option(WITH_LIBXML2 "Choose if libxml2 support should be built " FALSE)
if (WITH_LIBXML2 OR
    (BUILD_PLUGIN_OCI OR
     BUILD_PLUGIN_PGPOINTCLOUD OR
     BUILD_PLUGIN_SQLITE))
    set(WITH_LIBXML2 TRUE)
else()
    set(WITH_LIBXML2 FALSE)
endif()

if(WITH_LIBXML2)
    find_package(LibXml2)
    set_package_properties(LibXml2 PROPERTIES TYPE OPTIONAL)
    if(LIBXML2_FOUND)
        mark_as_advanced(CLEAR LIBXML2_INCLUDE_DIR)
        mark_as_advanced(CLEAR LIBXML2_LIBRARIES)
        include_directories(${LIBXML2_INCLUDE_DIR})
        set(PDAL_HAVE_LIBXML2 1)
    else()
        set(WITH_LIBXML2 FALSE)
        set(LIBXML2_LIBRARIES "")
    endif()
endif()

#------------------------------------------------------------------------------
# installation commands
#------------------------------------------------------------------------------

#install(FILES AUTHORS.txt LICENSE.txt
#  DESTINATION ${PDAL_DATA_DIR}/doc)

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


#------------------------------------------------------------------------------
# subdirectory controls
#------------------------------------------------------------------------------

# PDAL_TARGET_OBJECTS is used to collect the driver object libraries
set(PDAL_TARGET_OBJECTS "")

add_subdirectory(io)
add_subdirectory(filters)
add_subdirectory(kernels)
add_subdirectory(src)

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
list(APPEND CPACK_SOURCE_IGNORE_FILES "Makefile")
list(APPEND CPACK_SOURCE_IGNORE_FILES "CMakeFiles")
list(APPEND CPACK_SOURCE_IGNORE_FILES "CTestTestfile.cmake")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/test/data/local/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/doc/doxygen/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/doc/build/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/doc/presentations/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/scripts/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/cmake/examples/")

include(CPack)

add_custom_target(dist COMMAND ${CMAKE_MAKE_PROGRAM} package_source)

feature_summary(WHAT ALL INCLUDE_QUIET_PACKAGES)
