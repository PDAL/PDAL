#
# top-level CMake configuration file for libPC
#
# (based originally on the libLAS files copyright Mateusz Loskot)

cmake_minimum_required(VERSION 2.6.0)
project(libPC)

#------------------------------------------------------------------------------
# internal cmake settings
#------------------------------------------------------------------------------

set(CMAKE_COLOR_MAKEFILE ON)

# Allow advanced users to generate Makefiles printing detailed commands
mark_as_advanced(CMAKE_VERBOSE_MAKEFILE)

# Path to additional CMake modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})


#------------------------------------------------------------------------------
# libPC general settings
#------------------------------------------------------------------------------



# the next three lines are the ONLY place in the entire libpc system where
# the version info is hard-coded
SET(LIBPC_VERSION_MAJOR "0")
SET(LIBPC_VERSION_MINOR "1")
SET(LIBPC_VERSION_PATCH "0")

set(LIBPC_VERSION_STRING "${LIBPC_VERSION_MAJOR}.${LIBPC_VERSION_MINOR}.${LIBPC_VERSION_PATCH}")
SET(LIBPC_LIB_SOVERSION ${LIBPC_VERSION_MAJOR})
SET(LIBPC_C_LIB_SOVERSION ${LIBPC_VERSION_MAJOR})


# Name of C++ library
if(WIN32)
  set(LIBPC_LIB_NAME libpc)
else()
  set(LIBPC_LIB_NAME pc)
endif()

set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)



#------------------------------------------------------------------------------
# misc user options
#------------------------------------------------------------------------------

# Choose package components
set(WITH_APPS TRUE CACHE BOOL "Choose if libPC utilities should be built")
set(WITH_TESTS TRUE CACHE BOOL "Choose if libPC unit tests should be built")

# Choose to use pkg-config or not
set(WITH_PKGCONFIG FALSE CACHE BOOL
  "Choose whether a pkgconfig file (libPC.pc) should be installed")

set(WITH_ENDIANAWARE FALSE CACHE BOOL "Choose whether or not libPC should do runtime endianness switching.  Note that this can slow things down considerably if enabled by default")
if (WITH_ENDIANAWARE)
    set(LIBPC_ENDIAN_AWARE 1)
endif()


#------------------------------------------------------------------------------
# test harness settings
#------------------------------------------------------------------------------

# Name of test suite runner
set(LIBPC_UNIT_TEST libpc_test)

# Enable CTest and submissions to libPC dashboard at CDash
# http://my.cdash.org/index.php?project=libPC
set(ENABLE_CTEST FALSE CACHE BOOL
  "Enable CTest to support submissions of results to CDash at http://cdash.org")



#------------------------------------------------------------------------------
# General build settings
#------------------------------------------------------------------------------

# note we default to debug mode
if(NOT MSVC_IDE)
  if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING 
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel" FORCE)
  endif()
  message(STATUS "Setting libPC build type - ${CMAKE_BUILD_TYPE}")
endif()

set(LIBPC_BUILD_TYPE ${CMAKE_BUILD_TYPE})

# TODO: Still testing the output paths --mloskot
set(LIBPC_BUILD_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# Output directory in which to build RUNTIME target files.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${LIBPC_BUILD_OUTPUT_DIRECTORY})

# Output directory in which to build LIBRARY target files
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBPC_BUILD_OUTPUT_DIRECTORY})

# Output directory in which to build ARCHIVE target files.
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIBPC_BUILD_OUTPUT_DIRECTORY}) 


#------------------------------------------------------------------------------
# Platform and compiler specific settings
#------------------------------------------------------------------------------

if(WIN32)
  if (MSVC)
    if (MSVC80 OR MSVC90 OR MSVC10)
      add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
      add_definitions(-D_CRT_SECURE_NO_WARNINGS)
      add_definitions(-D_CRT_NONSTDC_NO_WARNING)
      add_definitions(-D_SCL_SECURE_NO_WARNINGS)
    endif()
    
    if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
      string(REGEX REPLACE "/W[0-4]" "/W4"
        CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    endif()
  endif()
  
  set(CMAKE_INCLUDE_PATH c:/osgeo4w/include;$ENV{CMAKE_INCLUDE_PATH})
  set(CMAKE_LIBRARY_PATH c:/osgeo4w/lib;$ENV{CMAKE_LIBRARY_PATH})

else()
  # not WIN32

  # Recommended C++ compilation flags
  # -Weffc++
  set(LIBPC_COMMON_CXX_FLAGS
    "-pedantic -ansi -Wall -Wpointer-arith -Wcast-align -Wcast-qual -Wfloat-equal -Wredundant-decls -Wno-long-long")

  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LIBPC_COMMON_CXX_FLAGS}")
    if (CMAKE_COMPILER_IS_GNUCXX)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++98")
    endif()

  elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" OR "${CMAKE_CXX_COMPILER}" MATCHES "clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LIBPC_COMMON_CXX_FLAGS}")
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

# Default to using static, multithreaded libraries for 
# linking under MSVC.  This is because we show users how to 
# use boostpro.com installer and install those options when linking 
# on windows in the compilation documentation.
if(WIN32)
  if (MSVC)
    set(Boost_USE_STATIC_LIBS   ON)
    set(Boost_USE_MULTITHREADED ON)
  endif(MSVC)
endif(WIN32)

# BUG: are there other Boost components we're using?
find_package(Boost 1.44 COMPONENTS program_options thread iostreams filesystem system unit_test_framework REQUIRED)

if(Boost_FOUND AND Boost_PROGRAM_OPTIONS_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

# make these available for the user to set.
mark_as_advanced(CLEAR Boost_INCLUDE_DIR) 
mark_as_advanced(CLEAR Boost_LIBRARY_DIRS) 
link_directories(${Boost_LIBRARY_DIRS}) 



#------------------------------------------------------------------------------
# settings for GDAL & related dependencies
#------------------------------------------------------------------------------
   
# GDAL/OGR support - optional, default=OFF
set(WITH_GDAL FALSE CACHE BOOL "Choose if GDAL support should be built")

if(WITH_GDAL)
  find_package(GDAL 1.6.0)
  set(LIBPC_HAVE_GDAL 1)
  include_directories(${GDAL_INCLUDE_DIR})
  mark_as_advanced(CLEAR GDAL_INCLUDE_DIR) 
  mark_as_advanced(CLEAR GDAL_LIBRARY) 
endif()

# GeoTIFF support - optional, default=OFF
set(WITH_GEOTIFF FALSE CACHE BOOL "Choose if GeoTIFF support should be built")

if(WITH_GEOTIFF)
  find_package(GeoTIFF 1.2.5)
  if(GEOTIFF_FOUND)
    include(CheckFunctionExists) 
    set(CMAKE_REQUIRED_LIBRARIES ${GEOTIFF_LIBRARY})
    include_directories(${GEOTIFF_INCLUDE_DIR})
    set(LIBPC_HAVE_LIBGEOTIFF 1)
  endif()

  # Determine libTIFF availability, required
  if (GEOTIFF_FOUND)
    mark_as_advanced(CLEAR TIFF_INCLUDE_DIR) 
    mark_as_advanced(CLEAR TIFF_LIBRARY) 
    if (WIN32)
        set(TIFF_NAMES libtiff_i)
    endif(WIN32)
    find_package(TIFF) 
    if (TIFF_FOUND)
      include_directories(${TIFF_INCLUDE_DIR})
    endif()
  endif()
endif()

if(WITH_GDAL)
  if (NOT GEOTIFF_FOUND)
    message(FATAL_ERROR
      "GDAL support requires GeoTIFF library which was not selected")
  endif()
endif()



#------------------------------------------------------------------------------
# settings for other 3rd party dependencies
#------------------------------------------------------------------------------

# swig support
if (WIN32)
  set(WITH_SWIG_CSHARP TRUE CACHE BOOL "Choose if you want to make C# bindings via Swig")
else()
  set(WITH_SWIG_CSHARP FALSE CACHE BOOL "Choose if you want to make C# bindings via Swig")
endif()

set(WITH_SWIG_PYTHON FALSE CACHE BOOL "Choose if you want to make Python bindings via Swig")

if(WITH_SWIG_CSHARP OR WITH_SWIG_PYTHON)
    find_package(swig 2.0.1 REQUIRED)
    include(${SWIG_USE_FILE})
    message(STATUS "Found Swig version ${SWIG_VERSION}")
    if(WITH_SWIG_CSHARP)
        add_subdirectory(csharp)
        set(LIBPC_HAVE_SWIG_CSHARP 1)
        message(STATUS "Enabling swig for csharp")
    endif()
    if(WITH_SWIG_PYTHON)
        add_subdirectory(python)
        set(LIBPC_HAVE_SWIG_PYTHON 1)
        message(STATUS "Enabling swig for python")
    endif()
endif()

# libxml2 support - optional, default=ON
set(WITH_LIBXML2 FALSE CACHE BOOL "Choose if libxml2 support should be built ")

if(WITH_LIBXML2)
    find_package(LibXml2)
    mark_as_advanced(CLEAR LIBXML2_INCLUDE_DIR)
    mark_as_advanced(CLEAR LIBXML2_LIBRARIES)
    if(LIBXML2_FOUND)
        include_directories(${LIBXML2_INCLUDE_DIR})
        set(LIBPC_HAVE_LIBXML2 1)
    endif()
endif()

# Oracle support - optional, default=OFF
set(WITH_ORACLE FALSE CACHE BOOL "Choose if Oracle support should be built")

if(WITH_ORACLE)
  find_package(Oracle)

  if(ORACLE_FOUND)
    include_directories(${ORACLE_INCLUDE_DIR})
    add_definitions(-DHAVE_ORACLE=1)
  endif()
endif()

# LASZIP support - optional, default=OFF
set(WITH_LASZIP FALSE CACHE BOOL "Choose if LASzip support should be built")
set(WITH_STATIC_LASZIP FALSE CACHE BOOL "Choose if LASzip should be statically linked")
mark_as_advanced(WITH_STATIC_LASZIP)

if(WITH_LASZIP)
    find_package(LASzip 1.0.1)
    if(LASZIP_FOUND)    
        set(CMAKE_REQUIRED_LIBRARIES ${LASZIP_LIBRARY})
        include_directories(${LASZIP_INCLUDE_DIR})
        mark_as_advanced(CLEAR LASZIP_INCLUDE_DIR) 
        mark_as_advanced(CLEAR LASZIP_LIBRARY) 
        mark_as_advanced(CLEAR LASZIP_VERSION) 
        set(LIBPC_HAVE_LASZIP 1)
    endif()
endif()

# LIBLAS support - optional, default=OFF
set(WITH_LIBLAS FALSE CACHE BOOL "Choose if libLAS support should be built")
set(WITH_STATIC_LIBLAS FALSE CACHE BOOL "Choose if libLAS should be statically linked")
mark_as_advanced(WITH_STATIC_LIBLAS)

if(WITH_LIBLAS)
    find_package(LibLAS 1.6.0)
    if(LIBLAS_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES ${LIBLAS_LIBRARY})
        include_directories(${LIBLAS_INCLUDE_DIR})
        mark_as_advanced(CLEAR LIBLAS_INCLUDE_DIR) 
        mark_as_advanced(CLEAR LIBLAS_LIBRARY) 
        mark_as_advanced(CLEAR LIBLAS_VERSION) 
        set(LIBPC_HAVE_LIBLAS 1)
    endif()
endif()

if (NOT LIBLAS_FOUND)
  message(FATAL_ERROR
    "libPC currently requires a working libLAS installation 1.6.0 or greater")
endif()

# MrSID/LiDAR support - optiona, default=OFF
set(WITH_MRSID FALSE CACHE BOOL "Choose if MrSID/LiDAR support should be built")

if(WITH_MRSID)
    find_package(MrSID)
    if(MRSID_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES ${MRSID_LIBRARY})
        include_directories(${MRSID_INCLUDE_DIR})
        add_definitions(-DHAVE_MRSID=1)
    endif()
endif()


#------------------------------------------------------------------------------
# installation path settings
#------------------------------------------------------------------------------

if(WIN32)
  set(DEFAULT_LIB_SUBDIR lib)
  set(DEFAULT_DATA_SUBDIR .)
  set(DEFAULT_INCLUDE_SUBDIR include)

  if (MSVC)
    set(DEFAULT_BIN_SUBDIR bin)
  else()
    set(DEFAULT_BIN_SUBDIR .)
  endif()
else()
  # Common locatoins for Unix and Mac OS X
  set(DEFAULT_BIN_SUBDIR bin)
  set(DEFAULT_LIB_SUBDIR lib)
  set(DEFAULT_DATA_SUBDIR share/libpc)
  set(DEFAULT_INCLUDE_SUBDIR include)
endif()

# Locations are changeable by user to customize layout of libPC installation
# (default values are platform-specific)
set(LIBPC_BIN_SUBDIR ${DEFAULT_BIN_SUBDIR} CACHE STRING
  "Subdirectory where executables will be installed")
set(LIBPC_LIB_SUBDIR ${DEFAULT_LIB_SUBDIR} CACHE STRING
  "Subdirectory where libraries will be installed")
set(LIBPC_INCLUDE_SUBDIR ${DEFAULT_INCLUDE_SUBDIR} CACHE STRING
  "Subdirectory where header files will be installed")
set(LIBPC_DATA_SUBDIR ${DEFAULT_DATA_SUBDIR} CACHE STRING
  "Subdirectory where data will be installed")

# Mark *_SUBDIR variables as advanced and dedicated to use by power-users only.
mark_as_advanced(LIBPC_BIN_SUBDIR
  LIBPC_LIB_SUBDIR LIBPC_INCLUDE_SUBDIR LIBPC_DATA_SUBDIR)

# Full paths for the installation
set(LIBPC_BIN_DIR ${LIBPC_BIN_SUBDIR})
set(LIBPC_LIB_DIR ${LIBPC_LIB_SUBDIR})
set(LIBPC_INCLUDE_DIR ${LIBPC_INCLUDE_SUBDIR})
set(LIBPC_DATA_DIR ${LIBPC_DATA_SUBDIR})



#------------------------------------------------------------------------------
# installation commands
#------------------------------------------------------------------------------

install(FILES AUTHORS.txt LICENSE.txt
  DESTINATION ${LIBPC_DATA_DIR}/doc)



#------------------------------------------------------------------------------
# subdirectory controls
#------------------------------------------------------------------------------

add_subdirectory(src)

add_subdirectory(apps)

if(WITH_TESTS)
  message(STATUS "Enable libPC unit tests to build - done")
  enable_testing()

  if(ENABLE_CTEST)
    message(STATUS
      "Enable CTest to support submissions of results to CDash at http://cdash.org")
    cmake_minimum_required(VERSION 2.8.0)
    # Dashboard has been prepared for experiments
    # http://my.cdash.org/index.php?project=libPC
    include(CTest)
    message(STATUS
      "Enable CTest to support submissions of results to CDash at http://cdash.org - done")
  endif()

  add_subdirectory(test)
else()
  if(ENABLE_CTEST)
    message(WARNING
      "CTest support requested but WITH_TESTS option not specified to build of libPC unit tests")
  endif()
endif()



#------------------------------------------------------------------------------
# CPACK controls
#------------------------------------------------------------------------------

SET(CPACK_PACKAGE_VERSION_MAJOR ${LIBPC_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${LIBPC_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${LIBPC_VERSION_MINOR})

SET(CPACK_SOURCE_GENERATOR "TGZ;ZIP;TBZ2")
SET(CPACK_CMAKE_GENERATOR "Unix Makefiles")

set(CPACK_SOURCE_PACKAGE_FILE_NAME
  "${CMAKE_PROJECT_NAME}-${VERSION}")
  
list(APPEND CPACK_SOURCE_IGNORE_FILES "/test/data/TO_core_last_zoom.las")
list(APPEND CPACK_SOURCE_IGNORE_FILES "_CPack_Packages")
list(APPEND CPACK_SOURCE_IGNORE_FILES ".gz")
list(APPEND CPACK_SOURCE_IGNORE_FILES ".bz2")
list(APPEND CPACK_SOURCE_IGNORE_FILES ".hg")
list(APPEND CPACK_SOURCE_IGNORE_FILES "README")
list(APPEND CPACK_SOURCE_IGNORE_FILES "HOWTORELEASE.txt")
list(APPEND CPACK_SOURCE_IGNORE_FILES "/doc/_static/files")

list(APPEND CPACK_SOURCE_IGNORE_FILES "CMakeCache.txt")

list(APPEND CPACK_SOURCE_IGNORE_FILES "CPackConfig.cmake")
list(APPEND CPACK_SOURCE_IGNORE_FILES "schemas")
list(APPEND CPACK_SOURCE_IGNORE_FILES "hobu-config.bat")

list(APPEND CPACK_SOURCE_IGNORE_FILES "libpc-osgeo4w-init.bat")
list(APPEND CPACK_SOURCE_IGNORE_FILES "libpc-osgeo4w-start.bat.tmpl")

include(CPack)

#add_custom_target(dist COMMAND ${CMAKE_MAKE_PROGRAM} package_source)


#------------------------------------------------------------------------------
# finally, generate the libpc_config.h header
#------------------------------------------------------------------------------

configure_file(
  "${PROJECT_SOURCE_DIR}/libpc_defines.h.in"
  "${PROJECT_SOURCE_DIR}/include/libpc/libpc_defines.h")
