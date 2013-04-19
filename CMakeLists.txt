#
# top-level CMake configuration file for PDAL
#
# (based originally on the libLAS files copyright Mateusz Loskot)

if(WIN32)
cmake_minimum_required(VERSION 2.8.7)
else()
cmake_minimum_required(VERSION 2.6.0)
endif()

project(PDAL)

#------------------------------------------------------------------------------
# internal cmake settings
#------------------------------------------------------------------------------

set(CMAKE_COLOR_MAKEFILE ON)

# Allow advanced users to generate Makefiles printing detailed commands
mark_as_advanced(CMAKE_VERBOSE_MAKEFILE)

# Path to additional CMake modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})


#------------------------------------------------------------------------------
# PDAL general settings
#------------------------------------------------------------------------------



# the next three lines are the ONLY place in the entire pdal system where
# the version info is hard-coded
SET(PDAL_VERSION_MAJOR "0")
SET(PDAL_VERSION_MINOR "8")
SET(PDAL_VERSION_PATCH "6")

set(PDAL_VERSION_STRING "${PDAL_VERSION_MAJOR}.${PDAL_VERSION_MINOR}.${PDAL_VERSION_PATCH}")
SET(PDAL_LIB_SOVERSION ${PDAL_VERSION_MAJOR})
SET(PDAL_C_LIB_SOVERSION ${PDAL_VERSION_MAJOR})


# Name of C++ library
set(PDAL_LIB_NAME pdal)

set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)



#------------------------------------------------------------------------------
# misc user options
#------------------------------------------------------------------------------

# Choose package components
set(WITH_APPS TRUE CACHE BOOL "Choose if PDAL utilities should be built")
set(WITH_TESTS TRUE CACHE BOOL "Choose if PDAL unit tests should be built")

# Choose to use pkg-config or not
set(WITH_PKGCONFIG FALSE CACHE BOOL
  "Choose whether a pkgconfig file (PDAL.pc) should be installed")


#------------------------------------------------------------------------------
# test harness settings
#------------------------------------------------------------------------------

# Name of test suite runner
set(PDAL_UNIT_TEST pdal_test)

# Enable CTest and submissions to PDAL dashboard at CDash
# http://my.cdash.org/index.php?project=PDAL
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
  message(STATUS "Setting PDAL build type - ${CMAKE_BUILD_TYPE}")
endif()

set(PDAL_BUILD_TYPE ${CMAKE_BUILD_TYPE})

# TODO: Still testing the output paths --mloskot
set(PDAL_BUILD_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# Output directory in which to build RUNTIME target files.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PDAL_BUILD_OUTPUT_DIRECTORY})

# Output directory in which to build LIBRARY target files
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PDAL_BUILD_OUTPUT_DIRECTORY})

# Output directory in which to build ARCHIVE target files.
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PDAL_BUILD_OUTPUT_DIRECTORY}) 


#------------------------------------------------------------------------------
# Platform and compiler specific settings
#------------------------------------------------------------------------------

if(WIN32)
  if (MSVC)
    if (MSVC80 OR MSVC90 OR MSVC10 OR MSVC11)
      add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
      add_definitions(-D_CRT_SECURE_NO_WARNINGS)
      add_definitions(-D_CRT_NONSTDC_NO_WARNING)
      add_definitions(-D_SCL_SECURE_NO_WARNINGS)

      set(PDAL_COMPILER_MSVC 1)
      if (MSVC11)
        set(PDAL_COMPILER_VC11 1)
      elseif (MSVC10)
        set(PDAL_COMPILER_VC10 1)
      elseif (MSVC9)
        set(PDAL_COMPILER_VC9 1)
      elseif (MSVC8)
        set(PDAL_COMPILER_VC8 1)
      endif()
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

  set(PDAL_PLATFORM_WIN32 1)

else()
  # not WIN32

  # Recommended C++ compilation flags
  # -Weffc++
  set(PDAL_COMMON_CXX_FLAGS
	  "-Wextra -Wall -Wno-unused-parameter -Wno-unused-variable -Wpointer-arith -Wcast-align -Wcast-qual -Wfloat-equal -Wredundant-decls -Wno-long-long -fPIC")

  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PDAL_COMMON_CXX_FLAGS}")
    if (CMAKE_COMPILER_IS_GNUCXX)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++98")
    endif()
    
    set(PDAL_COMPILER_GCC 1)
    
  elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" OR "${CMAKE_CXX_COMPILER}" MATCHES "clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PDAL_COMMON_CXX_FLAGS}")
    set(PDAL_COMPILER_CLANG 1)
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

if (NOT PDAL_EMBED_BOOST)
  find_package(Boost 1.48 COMPONENTS program_options thread iostreams filesystem system unit_test_framework random)
endif()


if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})

  # make these available for the user to set.
  mark_as_advanced(CLEAR Boost_INCLUDE_DIR) 
  mark_as_advanced(CLEAR Boost_LIBRARY_DIRS) 
  link_directories(${Boost_LIBRARY_DIRS})

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
  
else()
  set(BOOST_ROOT ${PROJECT_SOURCE_DIR}/boost)

  # find_package would normally set the following variables
  set(Boost_FOUND TRUE)
  set(Boost_INCLUDE_DIRS ${BOOST_ROOT})

  set(PDALBOOST_LIB_NAME "pdalboost" CACHE STRING "use embedded boost")
  mark_as_advanced(FORCE PDALBOOST_LIB_NAME)

  if (WIN32)
    set(Boost_LIBRARIES "${PDALBOOST_LIB_NAME}.lib")
    link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
  else()
    # Some Cmake versions are borken here. See
    # http://www.cmake.org/Bug/view.php?id=12258#c26851
    # set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-isystem")
    # include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIRS}/)
    set(Boost_LIBRARIES "${PDALBOOST_LIB_NAME}")
  endif()

  # this prevents auto-linking pragmas from depending on wrong lib names
  add_definitions(-DBOOST_ALL_NO_LIB)

  include_directories( ${Boost_INCLUDE_DIRS}/)
  set(PDAL_EMBED_BOOST TRUE CACHE BOOL "use embedded boost")
  add_subdirectory(boost)
  message(STATUS "Using embedded boost tree")
endif(Boost_FOUND)

#------------------------------------------------------------------------------
# settings for GDAL & related dependencies
#------------------------------------------------------------------------------
   
# GDAL/OGR support - optional, default=OFF
set(WITH_GDAL FALSE CACHE BOOL "Choose if GDAL support should be built")

if(WITH_GDAL)
  find_package(GDAL 1.9.0)
  set(PDAL_HAVE_GDAL 1)
  include_directories(${GDAL_INCLUDE_DIR})
  mark_as_advanced(CLEAR GDAL_INCLUDE_DIR) 
  mark_as_advanced(CLEAR GDAL_LIBRARY)
  message(STATUS "...building with GDAL")

  find_package(GEOS 3.3)
  if (GEOS_FOUND)
    set(PDAL_HAVE_GEOS 1)
    include_directories(${GEOS_INCLUDE_DIR})
    message(STATUS "...GEOS found and used in addition to GDAL")
  else()
    set(GEOS_LIBRARY "") 
  endif()

else()
  set(GDAL_LIBRARY "") 
endif()

# GeoTIFF support - optional, default=OFF
set(WITH_GEOTIFF FALSE CACHE BOOL "Choose if GeoTIFF support should be built")

if(WITH_GEOTIFF)
  find_package(GeoTIFF 1.2.5)
  if(GEOTIFF_FOUND)
    include(CheckFunctionExists) 
    set(CMAKE_REQUIRED_LIBRARIES ${GEOTIFF_LIBRARY})
    include_directories(${GEOTIFF_INCLUDE_DIR})
    set(PDAL_HAVE_LIBGEOTIFF 1)
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

set(WITH_FLANN FALSE CACHE BOOL "Choose if FLANN support should be built")
if (WITH_FLANN)
  find_package(Flann 1.7)
  if (FLANN_FOUND)
    mark_as_advanced(CLEAR FLANN_INCLUDE_DIR) 
    mark_as_advanced(CLEAR FLANN_LIBRARY) 
    include_directories(${FLANN_INCLUDE_DIR})
    set(PDAL_HAVE_FLANN 1)
  endif()
endif()

if(WITH_GDAL)
  if (NOT GEOTIFF_FOUND)
    message(FATAL_ERROR
      "GDAL support requires GeoTIFF library which was not selected")
  endif()
endif()

set(WITH_SOCI FALSE CACHE BOOL "Choose if SOCI database support should be built")
if (WITH_SOCI)
find_package(Soci 3.1.0)
if (SOCI_FOUND)
  mark_as_advanced(CLEAR SOCI_INCLUDE_DIRS) 
  mark_as_advanced(CLEAR SOCI_LIBRARY) 
  include_directories(${SOCI_INCLUDE_DIRS})
  set(PDAL_HAVE_SOCI 1)
endif()
endif()


#------------------------------------------------------------------------------
# settings for other 3rd party dependencies
#------------------------------------------------------------------------------

# swig support
if (WIN32)
    # keep C# bindings it off by default, even on windows, since not everyone wants it
    set(WITH_SWIG_CSHARP FALSE CACHE BOOL "Choose if you want to make C# bindings via Swig")
else()
    # swig/C# not likely to be supported anytime soon
    set(WITH_SWIG_CSHARP FALSE CACHE BOOL "Choose if you want to make C# bindings via Swig")
endif()

set(WITH_SWIG_PYTHON FALSE CACHE BOOL "Choose if you want to make Python bindings via Swig")

if(WITH_SWIG_CSHARP OR WITH_SWIG_PYTHON)
    find_package(swig 2.0.1 REQUIRED)
    if (SWIG_FOUND)
        include(${SWIG_USE_FILE})
        message(STATUS "Found Swig version ${SWIG_VERSION}")
        if(WITH_SWIG_CSHARP)
            add_subdirectory(csharp)
            set(PDAL_HAVE_SWIG_CSHARP 1)
            message(STATUS "...enabling swig for csharp")
        endif()
        if(WITH_SWIG_PYTHON)
            add_subdirectory(python)
            set(PDAL_HAVE_SWIG_PYTHON 1)
            message(STATUS "Enabling swig for python")
        endif()
    endif()
endif()

# iconv support - optional, default=OFF
# GDAL and libxml2 require iconv.  The library is supplied by OSGeo4W.
set(WITH_ICONV FALSE CACHE BOOL "Choose if IConv support should be built")
if(WITH_ICONV)
    find_package(ICONV)
    mark_as_advanced(CLEAR ICONV_INCLUDE_DIR)
    mark_as_advanced(CLEAR ICONV_LIBRARIES)
    if(ICONV_FOUND)
        include_directories(${ICONV_INCLUDE_DIR})
        set(PDAL_HAVE_ICONV 1)
    endif()
endif()


# libxml2 support - optional, default=OFF
set(WITH_LIBXML2 FALSE CACHE BOOL "Choose if libxml2 support should be built ")

if(WITH_LIBXML2)
    find_package(LibXml2)
    mark_as_advanced(CLEAR LIBXML2_INCLUDE_DIR)
    mark_as_advanced(CLEAR LIBXML2_LIBRARIES)
    if(LIBXML2_FOUND)
        include_directories(${LIBXML2_INCLUDE_DIR})
        set(PDAL_HAVE_LIBXML2 1)
    endif()
endif()

if(NOT LIBXML2_FOUND)
  message(STATUS "libxml2 not selected or not found, pcinfo will not be build")
endif()

# Oracle support - optional, default=OFF
set(WITH_ORACLE FALSE CACHE BOOL "Choose if Oracle support should be built")

if(WITH_ORACLE)
  find_package(Oracle)

  if(ORACLE_FOUND)
    include_directories(${ORACLE_INCLUDE_DIR})
    set(PDAL_HAVE_ORACLE 1)
    message(STATUS "...building with Oracle")
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
        set(PDAL_HAVE_LASZIP 1)
    endif()
endif()


# MrSID/LiDAR support - optiona, default=OFF
set(WITH_MRSID FALSE CACHE BOOL "Choose if MrSID/LiDAR support should be built")

if(WITH_MRSID)

    set(MRSID_ROOT "/Users/hobu/installs/mrsid/Lidar_DSDK" CACHE STRING "Root directory of MrSID install")

    find_package(MrSID)
    if(MRSID_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES ${MRSID_LIBRARY})
        include_directories(${MRSID_INCLUDE_DIR})
        add_definitions(-DHAVE_MRSID=1)
        set(PDAL_HAVE_MRSID 1)
    endif()
endif()


# CARIS/BDB support - optional, default=OFF
set(WITH_CARIS FALSE CACHE BOOL "Choose if CARIS/BDB support should be built")
if(WITH_CARIS)
     set(CARIS_FOUND 1)
#    set(MRSID_ROOT "/Users/hobu/installs/mrsid/Lidar_DSDK" CACHE STRING "Root directory of MrSID install")
#    find_package(MrSID)
#    if(MRSID_FOUND)
#        set(CMAKE_REQUIRED_LIBRARIES ${MRSID_LIBRARY})
#        include_directories(${MRSID_INCLUDE_DIR})
#        add_definitions(-DHAVE_MRSID=1)
        set(PDAL_HAVE_CARIS 1)
#    endif()
endif()


# Points2Grid
# Points2Grid support - optional, default=OFF
set(WITH_P2G FALSE CACHE BOOL "Choose if Points2Grid support should be built")

if(WITH_P2G)
    find_package(Points2Grid)
    if(P2G_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES ${P2G_LIBRARY})
        include_directories(${P2G_INCLUDE_DIR})
        add_definitions(-DHAVE_P2G=1)
        set(PDAL_HAVE_P2G 1)
    endif()
endif()


if(WITH_FREEGLUT)
  find_package(FreeGLUT)
  if(FREEGLUT_FOUND)
    include_directories(${FREEGLUT_INCLUDE_DIR})
    add_definitions(-DHAVE_FREEGLUT=1)
    set(PDAL_HAVE_FREEGLUT 1)
    message(STATUS "Found FreeGLUT")
  endif()
endif()

set (WITH_HEXER FALSE CACHE BOOL "Whether or not hexbin filter is built")

if(WITH_HEXER)
  find_package(Hexer)
  if(HEXER_FOUND)
    include_directories(${HEXER_INCLUDE_DIR})
    add_definitions(-DHAVE_HEXER=1)
    set(PDAL_HAVE_HEXER 1)
  endif()
  
endif()

# default is false for now
set(WITH_PYTHON FALSE CACHE BOOL "Choose if Python support (for PLang filters) should be built")
if(WITH_PYTHON)
  FIND_PACKAGE(PythonInterp)
  find_package(PythonLibs)
  if(PYTHONLIBS_FOUND)
    set(CMAKE_REQUIRED_LIBRARIES ${PYTHON_LIBRARY})
    include_directories(${PYTHON_INCLUDE_DIR})
    add_definitions(-DHAVE_PYTHON=1)    
    set(PDAL_HAVE_PYTHON 1)
    
    find_package(NumPy 1.5)
    include_directories(${NUMPY_INCLUDE_DIR})
    message(STATUS "Found Python: ${PYTHON_LIBRARY}")
  endif()
endif()

set (USE_PDAL_PLUGIN_TEXT FALSE CACHE BOOL "Build the text driver as a plugin rather than embedding")
set (USE_PDAL_PLUGIN_SOCI FALSE CACHE BOOL "Build the soci driver as a plugin rather than embedding")
set (USE_PDAL_PLUGIN_OCI FALSE CACHE BOOL "Build the oci driver as a plugin rather than embedding")
set (USE_PDAL_PLUGIN_MRSID FALSE CACHE BOOL "Build the MrSID driver as a plugin rather than embedding")
set (USE_PDAL_PLUGIN_CARIS FALSE CACHE BOOL "Build the Caris driver as a plugin rather than embedding")



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
  # Common Locations for Unix and Mac OS X
  set(DEFAULT_BIN_SUBDIR bin)
  set(DEFAULT_LIB_SUBDIR lib)
  set(DEFAULT_DATA_SUBDIR share/pdal)
  set(DEFAULT_INCLUDE_SUBDIR include)
endif()

# Locations are changeable by user to customize layout of PDAL installation
# (default values are platform-specific)
set(PDAL_BIN_SUBDIR ${DEFAULT_BIN_SUBDIR} CACHE STRING
  "Subdirectory where executables will be installed")
set(PDAL_LIB_SUBDIR ${DEFAULT_LIB_SUBDIR} CACHE STRING
  "Subdirectory where libraries will be installed")
set(PDAL_INCLUDE_SUBDIR ${DEFAULT_INCLUDE_SUBDIR} CACHE STRING
  "Subdirectory where header files will be installed")
set(PDAL_DATA_SUBDIR ${DEFAULT_DATA_SUBDIR} CACHE STRING
  "Subdirectory where data will be installed")

# Mark *_SUBDIR variables as advanced and dedicated to use by power-users only.
mark_as_advanced(PDAL_BIN_SUBDIR
  PDAL_LIB_SUBDIR PDAL_INCLUDE_SUBDIR PDAL_DATA_SUBDIR)

# Full paths for the installation
set(PDAL_BIN_DIR ${PDAL_BIN_SUBDIR})
set(PDAL_LIB_DIR ${PDAL_LIB_SUBDIR})
set(PDAL_INCLUDE_DIR ${PDAL_INCLUDE_SUBDIR})
set(PDAL_DATA_DIR ${PDAL_DATA_SUBDIR})



#------------------------------------------------------------------------------
# installation commands
#------------------------------------------------------------------------------

install(FILES AUTHORS.txt LICENSE.txt
  DESTINATION ${PDAL_DATA_DIR}/doc)

#------------------------------------------------------------------------------
# generate the pdal_config.h header
#------------------------------------------------------------------------------

configure_file(
  "${PROJECT_SOURCE_DIR}/pdal_defines.h.in"
  "${PDAL_INCLUDE_DIR}/pdal/pdal_defines.h")

#------------------------------------------------------------------------------
# subdirectory controls
#------------------------------------------------------------------------------

add_subdirectory(src)

add_subdirectory(apps)

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



#------------------------------------------------------------------------------
# CPACK controls
#------------------------------------------------------------------------------

SET(CPACK_PACKAGE_VERSION_MAJOR ${PDAL_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${PDAL_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${PDAL_VERSION_MINOR})

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

list(APPEND CPACK_SOURCE_IGNORE_FILES "pdal-osgeo4w-init.bat")
list(APPEND CPACK_SOURCE_IGNORE_FILES "pdal-osgeo4w-start.bat.tmpl")

include(CPack)

#add_custom_target(dist COMMAND ${CMAKE_MAKE_PROGRAM} package_source)



