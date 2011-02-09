###############################################################################
# Main CMake configuration file for libLAS
#
# Author: Mateusz Loskot <mateusz@loskot.net>
#
#
###############################################################################
# libLAS general settings
project(libLAS)


SET(LIBLAS_VERSION_MAJOR "1")
SET(LIBLAS_VERSION_MINOR "6")
SET(LIBLAS_VERSION_PATCH "0")
set(VERSION "${LIBLAS_VERSION_MAJOR}.${LIBLAS_VERSION_MINOR}.${LIBLAS_VERSION_PATCH}")
SET(LIBLAS_LIB_SOVERSION "2.0.0")
SET(LIBLAS_C_LIB_SOVERSION "2.0.0")
SET(OSGEO4W_UPSTREAM_RELEASE "1")

if(WIN32)
  # Name of C++ library
  set(LIBLAS_LIB_NAME liblas)

  # Name of C library
  set(LIBLAS_C_LIB_NAME liblas_c)
  
  # Name of Chipper library
  set(LASCHIP_LIB_NAME liblas_chipper )  
  
  # Name of Kernel libray
  set(LASKERNEL_LIB_NAME liblas_kernel )
  
else()
  # Name of C++ library
  set(LIBLAS_LIB_NAME las)
  
  # Name of C library
  set(LIBLAS_C_LIB_NAME las_c)

  # Name of Chipper library
  set(LASCHIP_LIB_NAME las_chipper )  

  # Name of Kernel libray
  set(LASKERNEL_LIB_NAME las_kernel )
endif()

# Name of test suite runner
set(LIBLAS_UNIT_TEST liblas_test)

set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

# Choose package components
set(WITH_UTILITIES TRUE CACHE BOOL "Choose if libLAS utilities should be built")
set(WITH_TESTS TRUE CACHE BOOL "Choose if libLAS unit tests should be built")

# Choose to use pkg-config or not
set(WITH_PKGCONFIG FALSE CACHE BOOL
  "Choose whether a pkgconfig file (libLAS.pc) should be installed")

set(WITH_ENDIANAWARE FALSE CACHE BOOL "Choose whether or not libLAS should do runtime endianness switching.  Note that this can slow things down considerably if enabled by default")

# Enable CTest and submissions to libLAS dashboard at CDash
# http://my.cdash.org/index.php?project=libLAS
set(ENABLE_CTEST FALSE CACHE BOOL
  "Enable CTest to support submissions of results to CDash at http://cdash.org")

###############################################################################
# CMake settings
cmake_minimum_required(VERSION 2.6.0)

set(CMAKE_COLOR_MAKEFILE ON)

# Allow advanced users to generate Makefiles printing detailed commands
mark_as_advanced(CMAKE_VERBOSE_MAKEFILE)

# Path to additional CMake modules
set(CMAKE_MODULE_PATH ${libLAS_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})

###############################################################################
# General build settings

# Default to debug mode until final 1.6.0 release 
if(NOT MSVC_IDE)
  if(NOT CMAKE_BUILD_TYPE)

  set(CMAKE_BUILD_TYPE Debug CACHE STRING 
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel" FORCE)
  endif()
    message(STATUS "Setting libLAS build type - ${CMAKE_BUILD_TYPE}")
endif()

# TODO: Still testing the output paths --mloskot
set(LIBLAS_BUILD_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})

# Output directory in which to build RUNTIME target files.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${LIBLAS_BUILD_OUTPUT_DIRECTORY})

# Output directory in which to build LIBRARY target files
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBLAS_BUILD_OUTPUT_DIRECTORY})

# Output directory in which to build ARCHIVE target files.
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIBLAS_BUILD_OUTPUT_DIRECTORY}) 

file(READ "doc/index.txt" README )
file(WRITE "README.txt" "${README}")

###############################################################################
# Platform and compiler specific settings

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

    # Generate dot-user file with user-specific settings for
    # Visual Studio project
    set(MSVC_ENVIRONMENT_PATH "" CACHE STRING
      "Custom PATH for Environment property in Visual Studio for project")
    mark_as_advanced(MSVC_ENVIRONMENT_PATH)
    set(VCPROJ_USER_ENVIRONMENT_PATH_DEBUG "${ENVIRONMENT_PATH}")
    mark_as_advanced(VCPROJ_USER_ENVIRONMENT_PATH_DEBUG)

  endif()
  
  set(CMAKE_INCLUDE_PATH c:/osgeo4w/include;$ENV{CMAKE_INCLUDE_PATH})
  set(CMAKE_LIBRARY_PATH c:/osgeo4w/lib;$ENV{CMAKE_LIBRARY_PATH})
else()

  # Recommended C++ compilation flags
  # -Weffc++
  set(LIBLAS_COMMON_CXX_FLAGS
	"-pedantic -ansi -Wall -Wpointer-arith -Wcast-align -Wcast-qual -Wfloat-equal -Wredundant-decls -Wno-long-long")

  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LIBLAS_COMMON_CXX_FLAGS}")
    if (CMAKE_COMPILER_IS_GNUCXX)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++98")
    endif()

  elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" OR "${CMAKE_CXX_COMPILER}" MATCHES "clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LIBLAS_COMMON_CXX_FLAGS}")
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

###############################################################################
# Search for dependencies

# Boost C++ Libraries support - required
message(STATUS "Searching for Boost 1.38+ - done")

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

# NOTE: Add iostreams to COMPONENTS list to enable bigfile_boost_iostreams_test
find_package(Boost 1.38 COMPONENTS program_options thread REQUIRED)

if(Boost_FOUND AND Boost_PROGRAM_OPTIONS_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

# make these available for the user to set.
mark_as_advanced(CLEAR Boost_INCLUDE_DIR) 
mark_as_advanced(CLEAR Boost_LIBRARY_DIRS) 
link_directories(${Boost_LIBRARY_DIRS}) 


	
# libxml2 support - optional, default=ON
set(WITH_LIBXML2 FALSE CACHE BOOL "Choose if libxml2 support should be built ")

if(WITH_LIBXML2)
    find_package(LibXml2)
    mark_as_advanced(CLEAR LIBXML2_INCLUDE_DIR)
    mark_as_advanced(CLEAR LIBXML2_LIBRARIES)
    if(LIBXML2_FOUND)
        include_directories(${LIBXML2_INCLUDE_DIR})
        add_definitions(-DHAVE_LIBXML2=1)
    endif()
endif()

# GDAL/OGR support - optional, default=OFF
set(WITH_GDAL FALSE CACHE BOOL "Choose if GDAL support should be built")

if(WITH_GDAL)

  find_package(GDAL 1.6.0)

  # if(GDAL_FOUND)
  #   # Confirm required API is available
  #   include(CheckFunctionExists) 
  #   set(CMAKE_REQUIRED_LIBRARIES ${GDAL_LIBRARY})
  #   check_function_exists(OSRSetEquirectangular2 HAVE_OSRSETEQUIRECTANGULAR2)
  #   
  #   if(NOT HAVE_OSRSETEQUIRECTANGULAR2)
  #     # Reset to NOT found for GDAL/OGR
  #     set(GDAL_FOUND)
  #     message(FATAL_ERROR
  #       "Unable to find OSRSetEquirectangular2 within GDAL library at ${GDAL_INCLUDE_DIR} ${GDAL_LIBRARY}")
  #   else()
      include_directories(${GDAL_INCLUDE_DIR})
      add_definitions(-DHAVE_GDAL=1)
  #   endif()
  # endif()

  mark_as_advanced(CLEAR GDAL_INCLUDE_DIR) 
  mark_as_advanced(CLEAR GDAL_LIBRARY) 

endif()

# GeoTIFF support - optional, default=OFF
set(WITH_GEOTIFF FALSE CACHE BOOL "Choose if GeoTIFF support should be built")

if(WITH_GEOTIFF)
  find_package(GeoTIFF 1.2.5)

  if(GEOTIFF_FOUND)
    
    # Confirm required API is available
    include(CheckFunctionExists) 
    set(CMAKE_REQUIRED_LIBRARIES ${GEOTIFF_LIBRARY})
    #       check_function_exists(ST_Create HAVE_ST_CREATE)
    
    #        if(NOT HAVE_ST_CREATE)
    #            set(GEOTIFF_FOUND) # Reset to NOT found for GeoTIFF
    #            message(FATAL_ERROR "GeoTIFF support requires libgeotiff 1.2.5 or newer.")
    #        else()
    include_directories(${GEOTIFF_INCLUDE_DIR})
    add_definitions(-DHAVE_LIBGEOTIFF=1)
    #        endif()
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
set(WITH_STATIC_LASZIP FALSE CACHE BOOL "Choose of LASzip should be statically linked")
mark_as_advanced(WITH_STATIC_LASZIP)

if(WITH_LASZIP)
  find_package(LASzip 1.0.0)

  if(LASZIP_FOUND)    
    set(CMAKE_REQUIRED_LIBRARIES ${LASZIP_LIBRARY})
    include_directories(${LASZIP_INCLUDE_DIR})
    add_definitions(-DHAVE_LASZIP=1)
    mark_as_advanced(CLEAR LASZIP_INCLUDE_DIR) 
    mark_as_advanced(CLEAR LASZIP_LIBRARY) 
    mark_as_advanced(CLEAR LASZIP_VERSION) 
  endif()
endif()

if(WITH_ENDIANAWARE)
    add_definitions(-DLIBLAS_ENDIAN_AWARE=1)
endif()
###############################################################################
# Installation settings

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
  set(DEFAULT_DATA_SUBDIR share/liblas)
  set(DEFAULT_INCLUDE_SUBDIR include)
endif()

# Locations are changeable by user to customize layout of libLAS installation
# (default values are platform-specific)
set(LIBLAS_BIN_SUBDIR ${DEFAULT_BIN_SUBDIR} CACHE STRING
  "Subdirectory where executables will be installed")
set(LIBLAS_LIB_SUBDIR ${DEFAULT_LIB_SUBDIR} CACHE STRING
  "Subdirectory where libraries will be installed")
set(LIBLAS_INCLUDE_SUBDIR ${DEFAULT_INCLUDE_SUBDIR} CACHE STRING
  "Subdirectory where header files will be installed")
set(LIBLAS_DATA_SUBDIR ${DEFAULT_DATA_SUBDIR} CACHE STRING
  "Subdirectory where data will be installed")

# Mark *_SUBDIR variables as advanced and dedicated to use by power-users only.
mark_as_advanced(LIBLAS_BIN_SUBDIR
  LIBLAS_LIB_SUBDIR LIBLAS_INCLUDE_SUBDIR LIBLAS_DATA_SUBDIR)

# Full paths for the installation
set(LIBLAS_BIN_DIR ${LIBLAS_BIN_SUBDIR})
set(LIBLAS_LIB_DIR ${LIBLAS_LIB_SUBDIR})
set(LIBLAS_INCLUDE_DIR ${LIBLAS_INCLUDE_SUBDIR})
set(LIBLAS_DATA_DIR ${LIBLAS_DATA_SUBDIR})

###############################################################################
# Installation commands

install(FILES AUTHORS COPYING INSTALL LICENSE.txt README.txt
  DESTINATION ${LIBLAS_DATA_DIR}/doc)

###############################################################################
# Processing of project directories

add_subdirectory(src)

if(WITH_UTILITIES)
  message(STATUS "Enable libLAS utilities to build - done")
  add_subdirectory(apps)
endif()

if(WITH_TESTS)
  message(STATUS "Enable libLAS unit tests to build - done")
  enable_testing()


  if(ENABLE_CTEST)
    message(STATUS
      "Enable CTest to support submissions of results to CDash at http://cdash.org")
    cmake_minimum_required(VERSION 2.8.0)
    # Dashboard has been prepared for experiments
    # http://my.cdash.org/index.php?project=libLAS
    include(CTest)
    message(STATUS
      "Enable CTest to support submissions of results to CDash at http://cdash.org - done")
  endif()

  add_subdirectory(test)
else()
  if(ENABLE_CTEST)
    message(WARNING
      "CTest support requested but WITH_TESTS option not specified to build of libLAS unit tests")
  endif()
endif()



# Version information
SET(CPACK_PACKAGE_VERSION_MAJOR ${LIBLAS_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${LIBLAS_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${LIBLAS_VERSION_MINOR})

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

list(APPEND CPACK_SOURCE_IGNORE_FILES "liblas-osgeo4w-init.bat")
list(APPEND CPACK_SOURCE_IGNORE_FILES "liblas-osgeo4w-start.bat.tmpl")
include(CPack)

add_custom_target(dist COMMAND ${CMAKE_MAKE_PROGRAM} package_source)

#if (WIN32)
#    include(BuildOSGeo4W)
#endif(WIN32)
