#
# top-level CMake configuration file for PDAL
#
# (based originally on the libLAS files copyright Mateusz Loskot)

cmake_minimum_required(VERSION 2.8.6)

project(PDAL)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

#------------------------------------------------------------------------------
# internal cmake settings
#------------------------------------------------------------------------------

set(CMAKE_COLOR_MAKEFILE ON)

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
set(PDAL_VERSION_STRING 0.9.9 CACHE STRING "PDAL version")

DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

SET(PDAL_LIB_SOVERSION ${PDAL_VERSION_MAJOR})
SET(PDAL_C_LIB_SOVERSION ${PDAL_VERSION_MAJOR})

# Name of C++ library
set(PDAL_LIB_NAME pdalcpp)

set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

set(PDAL_OUTPUT_LIB_DIR "${PDAL_BINARY_DIR}/${PDAL_LIB_DIR}")
set(PDAL_OUTPUT_BIN_DIR "${PDAL_BINARY_DIR}/${PDAL_BIN_DIR}")
file(MAKE_DIRECTORY "${PDAL_OUTPUT_LIB_DIR}")
file(MAKE_DIRECTORY "${PDAL_OUTPUT_BIN_DIR}")
if(WIN32)
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

# Choose package components
option(WITH_APPS "Choose if PDAL utilities should be built" TRUE)
option(BUILD_PLUGIN_GREYHOUND "Choose if Greyhound support should be built" FALSE)
option(BUILD_PLUGIN_ICEBRIDGE "Choose if Icebridge support should be built" FALSE)
option(BUILD_PLUGIN_MRSID "Choose if MrSID/LiDAR support should be built" FALSE)
option(BUILD_PLUGIN_NITF "Choose if NITF support should be built (only install supported is from http://github.com/hobu/nitro)" TRUE)
option(BUILD_PLUGIN_OCI "Choose if Oracle support should be built" TRUE)
option(BUILD_PLUGIN_P2G "Choose if Points2Grid support should be built" FALSE)
option(BUILD_PLUGIN_PCL "Choose if PCL support should be built" FALSE)
option(BUILD_PLUGIN_PGPOINTCLOUD "Choose if PostgreSQL PointCloud support should be built" TRUE)
option(BUILD_PLUGIN_SQLITE "Choose if sqlite database support should be built" FALSE)
option(WITH_TESTS "Choose if PDAL unit tests should be built" TRUE)

# Choose to use pkg-config or not
option(WITH_PKGCONFIG "Choose whether a pkgconfig file (PDAL.pc) should be installed" TRUE)

cmake_dependent_option(BUILD_PGPOINTCLOUD_TESTS "Choose if PostgreSQL PointCloud tests should be built" OFF BUILD_PLUGIN_PGPOINTCLOUD OFF)
cmake_dependent_option(BUILD_SQLITE_TESTS "Choose if SQLite tests should be built" OFF BUILD_PLUGIN_SQLITE OFF)
cmake_dependent_option(BUILD_OCI_TESTS "Choose if OCI tests should be built" OFF BUILD_PLUGIN_OCI OFF)


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
         set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PDAL_COMMON_CXX_FLAGS}")
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

# Default to using static, multithreaded libraries for
# linking under MSVC.  This is because we show users how to
# use boostpro.com installer and install those options when linking
# on windows in the compilation documentation.
if(WIN32)
  if (MSVC)
    set(Boost_USE_STATIC_LIBS   ON)
    set(Boost_USE_MULTITHREADED ON)

    if (PDAL_USE_STATIC_RUNTIME)
      set(Boost_USE_STATIC_RUNTIME ON)
    endif(PDAL_USE_STATIC_RUNTIME)
  endif(MSVC)
endif(WIN32)

find_package(Boost 1.53 COMPONENTS program_options thread iostreams filesystem system unit_test_framework random REQUIRED)

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
find_package(GDAL 1.9.0)
if (GDAL_FOUND)
    include_directories("${GDAL_INCLUDE_DIR}")
    mark_as_advanced(CLEAR GDAL_INCLUDE_DIR)
    mark_as_advanced(CLEAR GDAL_LIBRARY)
    message(STATUS "...building with GDAL")
else()
    message(FATAL_ERROR "GDAL support is required")
endif()


#
# GEOS (optional)
#
find_package(GEOS 3.3)
if (GEOS_FOUND)
    find_file(GEOS_VERSION_H version.h "${GEOS_INCLUDE_DIR}/geos")
    if ("${GEOS_VERSION_H}" STREQUAL "GEOS_VERSION_H-NOTFOUND")
        set(GEOS_LIBRARY "")   
        set(GEOS_FOUND FALSE)
        set(WITH_GEOS FALSE)
    else()
        include_directories("${GEOS_INCLUDE_DIR}")
        set(PDAL_HAVE_GEOS 1)
        message(STATUS "...building with GEOS")        
    endif()
else()
    set(GEOS_LIBRARY "")
endif()


#
# GeoTIFF support (optional)
#
option(WITH_GEOTIFF "Choose if GeoTIFF support should be built" TRUE)
if(WITH_GEOTIFF)
    find_package(GeoTIFF 1.3.0)
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
    find_package(LASzip 1.0.1)
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


#
# ZLIB support - required
#
find_package(ZLIB REQUIRED)
if(ZLIB_FOUND)
    set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES} "${ZLIB_LIBRARY}")
	include_directories(${ZLIB_INCLUDE_DIR})
	mark_as_advanced(CLEAR ZLIB_INCLUDE_DIR)
	mark_as_advanced(CLEAR ZLIB_LIBRARY)
	set(PDAL_HAVE_ZLIB 1)
endif()


#
# CARIS/BDB support (optional)
#
option(WITH_CARIS "Choose if CARIS/BDB support should be built" FALSE)
if(WITH_CARIS)
    set(CARIS_FOUND 1)
    set(PDAL_HAVE_CARIS 1)
endif()


#
# Hexer - optional
#
option(WITH_HEXER "Whether or not hexbin filter is built" FALSE)
if(WITH_HEXER)
    find_package(Hexer)
    if(HEXER_FOUND)
        include_directories(${HEXER_INCLUDE_DIR})
        add_definitions(-DHAVE_HEXER=1)
        set(PDAL_HAVE_HEXER 1)
    endif()
endif()


#
# Python - optional
#
option(WITH_PYTHON "Choose if Python support (for PLang filters) should be built" TRUE)
if(WITH_PYTHON)
    FIND_PACKAGE(PythonInterp)
    find_package(PythonLibs 2.4)
    if(PYTHONLIBS_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES "${PYTHON_LIBRARY}")
        include_directories(SYSTEM ${PYTHON_INCLUDE_DIR})
        add_definitions(-DHAVE_PYTHON=1)
        set(PDAL_HAVE_PYTHON 1)

        find_package(NumPy 1.5 REQUIRED)
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


option(USE_PDAL_PLUGIN_TEXT "Build the text driver as a plugin rather than embedding" FALSE)
option(USE_PDAL_PLUGIN_SOCI "Build the soci driver as a plugin rather than embedding" FALSE)
option(USE_PDAL_PLUGIN_MRSID "Build the MrSID driver as a plugin rather than embedding" FALSE)
option(USE_PDAL_PLUGIN_CARIS "Build the Caris driver as a plugin rather than embedding" FALSE)

if (USE_PDAL_PLUGIN_TEXT OR USE_PDAL_PLUGIN_SOCI OR
    USE_PDAL_PLUGIN_MRSID OR USE_PDAL_PLUGIN_CARIS)
  if (PDAL_BUILD_STATIC)
    message(SEND_ERROR "Cannot build separate plugins with statically compiled PDAL")
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

set(pdal_defines_h_in "${CMAKE_CURRENT_SOURCE_DIR}/pdal_defines.h.in")
set(pdal_defines_h "${CMAKE_CURRENT_BINARY_DIR}/include/pdal/pdal_defines.h")
configure_file(${pdal_defines_h_in} ${pdal_defines_h})
PDAL_ADD_INCLUDES("" "" ${pdal_defines_h})
include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")


#------------------------------------------------------------------------------
# subdirectory controls
#------------------------------------------------------------------------------

if(WITH_TESTS)
  message(STATUS "Building with PDAL unit tests")
  enable_testing()

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


add_subdirectory(src)
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

SET(CPACK_SOURCE_GENERATOR "TBZ2")
SET(CPACK_PACKAGE_VENDOR "PDAL Development Team")

set(CPACK_PACKAGE_FILE_NAME
  "${CMAKE_PROJECT_NAME}-${VERSION}")

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
list(APPEND CPACK_SOURCE_IGNORE_FILES "bin/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "Makefile")
list(APPEND CPACK_SOURCE_IGNORE_FILES "CMakeFiles")
list(APPEND CPACK_SOURCE_IGNORE_FILES "CTestTestfile.cmake")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/test/data/local/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/doc/doxygen/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/doc/build/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/doc/presentations/")
list(APPEND CPACK_SOURCE_IGNORE_FILES "pdal-s3cmd.conf")
list(APPEND CPACK_SOURCE_IGNORE_FILES "import-pdal-boost.sh")

include(CPack)

add_custom_target(dist COMMAND ${CMAKE_MAKE_PROGRAM} package_source)
