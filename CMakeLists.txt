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

option(PDAL_EMBED_BOOST "use embedded rather than system boost" FALSE)

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
option(WITH_TESTS "Choose if PDAL unit tests should be built" TRUE)

option(WITH_STUBS "Use stubs if libraries are not enabled" TRUE)
set(PDAL_STUB_DIR "${PDAL_SOURCE_DIR}/include/stubs")

# Choose to use pkg-config or not
option(WITH_PKGCONFIG "Choose whether a pkgconfig file (PDAL.pc) should be installed" TRUE)


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
        endif()
        set(PDAL_COMPILER_GCC 1)
    elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" OR
            "${CMAKE_CXX_COMPILER}" MATCHES "clang")
         set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PDAL_COMMON_CXX_FLAGS}")
         set(PDAL_COMPILER_CLANG 1)
         set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
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

if (NOT PDAL_EMBED_BOOST)
  find_package(Boost 1.55 COMPONENTS program_options thread iostreams filesystem system unit_test_framework random)
endif()

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})

  # make these available for the user to set.
  mark_as_advanced(CLEAR Boost_INCLUDE_DIR) 
  mark_as_advanced(CLEAR Boost_LIBRARY_DIRS) 
  link_directories(${Boost_LIBRARY_DIRS})
 
else()
  set(BOOST_ROOT "${PROJECT_SOURCE_DIR}/boost")

  # find_package would normally set the following variables
  set(Boost_FOUND TRUE)
  set(Boost_INCLUDE_DIRS "${BOOST_ROOT}")

  set(PDALBOOST_LIB_NAME "pdalboost" CACHE STRING "use embedded boost")
  mark_as_advanced(FORCE PDALBOOST_LIB_NAME)

  if (WIN32)
    set(Boost_LIBRARIES ${PDALBOOST_LIB_NAME})
  else()
    # Some Cmake versions are borken here. See
    # http://www.cmake.org/Bug/view.php?id=12258#c26851
    # set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-isystem")
    # include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIRS}/)
    set(Boost_LIBRARIES "${PDALBOOST_LIB_NAME}")
  endif()

  # this prevents auto-linking pragmas from depending on wrong lib names
  add_definitions(-DBOOST_ALL_NO_LIB)

  include_directories("${Boost_INCLUDE_DIRS}/")
  set(PDAL_EMBED_BOOST TRUE )
  add_subdirectory(boost)
  message(STATUS "External boost not found, using embedded boost tree")
endif(Boost_FOUND)


#------------------------------------------------------------------------------
# settings for PCL
#------------------------------------------------------------------------------

option(WITH_PCL "Choose if PCL support should be built" FALSE)
if(WITH_PCL)
    find_package(PCL 1.7)
    if(PCL_FOUND)
        set(PDAL_HAVE_PCL 1)
        set(WITH_PCL_TEST 1)
        include_directories(${PCL_INCLUDE_DIRS})
        link_directories(${PCL_LIBRARY_DIRS})
        add_definitions(${PCL_DEFINITIONS})
        if (NOT PDAL_EMBED_BOOST)
            # PCL's configuration clobbers Boost find_package - do it again
          find_package(Boost 1.52 COMPONENTS program_options thread iostreams filesystem system unit_test_framework random)
        endif()
        
        message(STATUS "...building with PCL")
    else()
        set(PCL_LIBRARIES "")
    endif()
elseif(WITH_STUBS)
    set(PDAL_HAVE_PCL 1)
    set(WITH_PCL_TEST 0)
    include_directories(${PDAL_STUB_DIR})
endif()

#------------------------------------------------------------------------------
# settings for GDAL & related dependencies
#------------------------------------------------------------------------------
   
# GDAL/OGR support - optional, default=ON
option(WITH_GDAL "Choose if GDAL support should be built" TRUE)
if(WITH_GDAL)
    find_package(GDAL 1.9.0)
    if (GDAL_FOUND)
        set(PDAL_HAVE_GDAL 1)
        include_directories("${GDAL_INCLUDE_DIR}")
        mark_as_advanced(CLEAR GDAL_INCLUDE_DIR) 
        mark_as_advanced(CLEAR GDAL_LIBRARY)
        message(STATUS "...building with GDAL")
    else()
        set(WITH_GDAL FALSE)
        set(GDAL_LIBRARY "")
    endif()

    find_package(GEOS 3.3)
    if (GEOS_FOUND)
        set(PDAL_HAVE_GEOS 1)
        include_directories("${GEOS_INCLUDE_DIR}")
        message(STATUS "...building with GEOS")
    else()
        set(GEOS_LIBRARY "")
    endif()

endif()

# GeoTIFF support - optional, default=ON
option(WITH_GEOTIFF "Choose if GeoTIFF support should be built" TRUE)
if(WITH_GEOTIFF)
    find_package(GeoTIFF 1.2.5)
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

if(WITH_GDAL)
  if (NOT GEOTIFF_FOUND)
    message(FATAL_ERROR
      "GDAL support requires GeoTIFF library which was not selected")
  endif()
endif()

option(WITH_SQLITE "Choose if sqlite database support should be built" FALSE)
if (WITH_SQLITE)
    find_package(SQLite3)
    mark_as_advanced(CLEAR SQLITE3_INCLUDE_DIR) 
    mark_as_advanced(CLEAR SQLITE3_LIBRARY)
    include_directories(${SQLITE3_INCLUDE_DIR})
    set(PDAL_HAVE_SQLITE 1)
endif()

option(WITH_PGPOINTCLOUD "Choose if PostgreSQL PointCloud support should be built" TRUE)
cmake_dependent_option(
    WITH_PGPOINTCLOUD_TESTS
    "Choose if PostgreSQL PointCloud tests should be built"
    OFF
    WITH_PGPOINTCLOUD
    OFF
    )
if (WITH_PGPOINTCLOUD)
    find_package(PostgreSQL)
    if (POSTGRESQL_FOUND)
        set(PDAL_HAVE_POSTGRESQL 1)
        mark_as_advanced(CLEAR POSTGRESQL_INCLUDE_DIR)
        mark_as_advanced(CLEAR POSTGRESQL_LIBRARIES)
        include_directories(${POSTGRESQL_INCLUDE_DIR})
        message(STATUS "..building with PgPointCloud")
    else()
        set(WITH_PGPOINTCLOUD FALSE)
        set(POSTGRESQL_LIBRARIES "")
    endif()
endif()

option(WITH_NITRO "Choose if Nitro support (only install supported is from http://github.com/hobu/nitro)" TRUE)
if(WITH_NITRO)
    find_package(Nitro 2.6)
    IF (NITRO_FOUND)
        set(PDAL_HAVE_NITRO 1)
        include_directories(${NITRO_INCLUDE_DIR})
        include_directories("${NITRO_INCLUDE_DIR}/nitro/c++")
        include_directories("${NITRO_INCLUDE_DIR}/nitro/c")
        add_definitions("-D_REENTRANT")
        if (WIN32)
            add_definitions("-DSIZEOF_SIZE_T=4")
            add_definitions("-DIMPORT_NITRO_API")
        else()
            add_definitions("-D__POSIX")
        endif()
        message(STATUS "...building with Nitro")
    else()
        set(WITH_NITRO FALSE)
        set(NITRO_LIBRARIES "")   
    endif()
endif()

option(WITH_MSGPACK "Choose if MessagePack support should be built" TRUE)
if(WITH_MSGPACK)
    find_package(MsgPack)
    if (MSGPACK_FOUND)
        set(PDAL_HAVE_MSGPACK 1)
        include_directories("${MsgPack_INCLUDE_DIRS}")
        message(STATUS "...building with MessagePack")
    else()
        set(WITH_MSGPACK FALSE)
        set(MsgPack_LIBRARIES "")
    endif()
endif()
#------------------------------------------------------------------------------
# settings for other 3rd party dependencies
#------------------------------------------------------------------------------

# swig support
if (WIN32)
    # keep C# bindings it off by default, even on windows, since not everyone wants it
    option(WITH_SWIG_CSHARP "Choose if you want to make C# bindings via Swig" FALSE)
else()
    # swig/C# not likely to be supported anytime soon
    option(WITH_SWIG_CSHARP "Choose if you want to make C# bindings via Swig" FALSE)
endif()

option(WITH_SWIG_PYTHON "Choose if you want to make Python bindings via Swig" FALSE)

if(WITH_SWIG_CSHARP OR WITH_SWIG_PYTHON)
    find_package(SWIG 2.0.1 REQUIRED)
    if (SWIG_FOUND)
        include("${SWIG_USE_FILE}")
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

# iconv support - optional, default=ON
# GDAL and libxml2 require iconv.  The library is supplied by OSGeo4W.
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


# libxml2 support - optional, default=ON
option(WITH_LIBXML2 "Choose if libxml2 support should be built " TRUE)
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

# Oracle support - optional, default=ON
option(WITH_ORACLE "Choose if Oracle support should be built" TRUE)
if(WITH_ORACLE)
    find_package(Oracle)
    if(ORACLE_FOUND)
        include_directories(${ORACLE_INCLUDE_DIR})
        set(PDAL_HAVE_ORACLE 1)
        message(STATUS "...building with Oracle")
    else()
        set(WITH_ORACLE FALSE)
        set(ORACLE_LIBRARY "")
    endif()
endif()

# LASZIP support - optional, default=OFF
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

# ZLIB support - optional, default=ON
option(WITH_ZLIB "Choose of ZLIB support should be built" TRUE)
if(WITH_ZLIB)
    find_package(ZLIB)
    if(ZLIB_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES "${ZLIB_LIBRARY}")
	include_directories(${ZLIB_INCLUDE_DIR})
	mark_as_advanced(CLEAR ZLIB_INCLUDE_DIR)
	mark_as_advanced(CLEAR ZLIB_LIBRARY)
	set(PDAL_HAVE_ZLIB 1)
    else()
        set(ZLIB_LIBRARIES "")
	set(WITH_ZLIB FALSE)
    endif()
endif()

# MrSID/LiDAR support - optiona, default=OFF
option(WITH_MRSID "Choose if MrSID/LiDAR support should be built" FALSE)

if(WITH_MRSID)

    set(MRSID_ROOT "/Users/hobu/installs/mrsid/Lidar_DSDK" CACHE STRING "Root directory of MrSID install")

    find_package(MrSID)
    if(MRSID_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES "${MRSID_LIBRARY}")
        include_directories(${MRSID_INCLUDE_DIR})
        add_definitions(-DHAVE_MRSID=1)
        set(PDAL_HAVE_MRSID 1)
    endif()
endif()


# CARIS/BDB support - optional, default=OFF
option(WITH_CARIS "Choose if CARIS/BDB support should be built" FALSE)
if(WITH_CARIS)
    set(CARIS_FOUND 1)
    set(PDAL_HAVE_CARIS 1)
endif()


# Points2Grid support - optional, default=OFF
option(WITH_P2G "Choose if Points2Grid support should be built" FALSE)
if(WITH_P2G)
    find_package(Points2Grid)
    if(P2G_FOUND)
        set(CMAKE_REQUIRED_LIBRARIES "${P2G_LIBRARY}")
        include_directories(${P2G_INCLUDE_DIR})
        add_definitions(-DHAVE_P2G=1)
        set(PDAL_HAVE_P2G 1)
    endif()
endif()

# HDF5 support - optional, default=OFF
option(WITH_HDF5 "Choose if HDF5 support should be built" FALSE)
if(WITH_HDF5)
    set (HDF5_FIND_COMPONENTS "CXX")
    find_package(HDF5 COMPONENTS CXX)
    if(HDF5_FOUND)
        include_directories(${HDF5_INCLUDE_DIR})
        add_definitions(-DHAVE_HDF5=1)
        set(PDAL_HAVE_HDF5 1)
        message(STATUS "Found HDF5")
    else()
        set(WITH_HDF5 FALSE)
        set(PDAL_HAVE_HDF5 0)
        message(STATUS "HDF5 not found")
    endif()
endif()

# Json support - optional, default=OFF
option(WITH_JSON "Choose if JsonCpp support should be built" FALSE)
if(WITH_JSON OR WITH_GREYHOUND)
    find_package(JSONCPP)
    if(JSONCPP_FOUND)
        include_directories(${JSONCPP_INCLUDE_DIR})
        add_definitions(-DHAVE_JSONCPP=1)
        set(PDAL_HAVE_JSONCPP 1)
        message(STATUS "Found Jsoncpp ${JSONCPP_INCLUDE_DIR} ${JSONCPP_LIBRARIES}")
    else()
        set(WITH_JSONCPP FALSE)
        set(PDAL_HAVE_JSONCPP 0)
        message(STATUS "Jsoncpp not found")
    endif()
endif()

# WebSocket++ support - optional, default=OFF
option(WITH_WEBSOCKETPP "Choose if WebSocket++ support should be built" FALSE)
if(WITH_WEBSOCKETPP OR WITH_GREYHOUND)
    find_package(WEBSOCKETPP)
    if(WEBSOCKETPP_FOUND)
        include_directories(${WEBSOCKETPP_INCLUDE_DIR})
        add_definitions(-D_WEBSOCKETPP_NULLPTR_TOKEN_=0)
        add_definitions(-DHAVE_WEBSOCKETPP=1)
        set(PDAL_HAVE_WEBSOCKETPP 1)
        message(STATUS "Found WebSocket++ ${WEBSOCKETPP_INCLUDE_DIR} ${WEBSOCKETPP_LIBRARIES}")
    else()
        set(WITH_WEBSOCKETPP FALSE)
        set(PDAL_HAVE_WEBSOCKETPP 0)
        message(STATUS "WebSocket++ not found")
    endif()
endif()

# Greyhound support - optional, default=OFF
option(WITH_GREYHOUND "Choose if Greyhound support should be built" FALSE)
if(WITH_GREYHOUND)
    if(WEBSOCKETPP_FOUND AND JSONCPP_FOUND)
        set(WITH_GREYHOUND TRUE)
        set(PDAL_HAVE_GREYHOUND 1)
        message(STATUS "Building with Greyhound support")
    else()
        set(WITH_GREYHOUND FALSE)
        set(PDAL_HAVE_GREYHOUND 0)
        message(STATUS "Greyhound dependencies not found")
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

option(WITH_HEXER "Whether or not hexbin filter is built" FALSE)
if(WITH_HEXER)
    find_package(Hexer)
    if(HEXER_FOUND)
        include_directories(${HEXER_INCLUDE_DIR})
        add_definitions(-DHAVE_HEXER=1)
        set(PDAL_HAVE_HEXER 1)
    endif()
endif()

# default is false for now
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

option(USE_PDAL_PLUGIN_TEXT "Build the text driver as a plugin rather than embedding" FALSE)
option(USE_PDAL_PLUGIN_SOCI "Build the soci driver as a plugin rather than embedding" FALSE)
option(USE_PDAL_PLUGIN_OCI "Build the oci driver as a plugin rather than embedding" FALSE)
option(USE_PDAL_PLUGIN_MRSID "Build the MrSID driver as a plugin rather than embedding" FALSE)
option(USE_PDAL_PLUGIN_CARIS "Build the Caris driver as a plugin rather than embedding" FALSE)
option(USE_PDAL_PLUGIN_NITRO "Build the NITF writer as a plugin rather than embedding" FALSE)

if (USE_PDAL_PLUGIN_TEXT OR USE_PDAL_PLUGIN_SOCI OR USE_PDAL_PLUGIN_OCI OR
    USE_PDAL_PLUGIN_MRSID OR USE_PDAL_PLUGIN_CARIS OR USE_PDAL_PLUGIN_NITRO)
  if (PDAL_BUILD_STATIC)
    message(SEND_ERROR "Cannot build separate plugins with statically compiled PDAL")
  endif()
endif()

# check for demangling support
try_compile(DEMANGLER_FOUND
    ${CMAKE_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}/cmake/check_demangle.cpp
)

if(DEMANGLER_FOUND)
    set(PDAL_HAVE_DEMANGLER 1)
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

add_subdirectory(src)

if(WITH_APPS)
  add_subdirectory(apps)
endif()

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



