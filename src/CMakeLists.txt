###############################################################################
#
# src/CMakeLists.txt controls building of libPC library
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
###############################################################################

# Collect dependencies configuration
if(GDAL_FOUND)
  set(LIBPC_GDAL_CPP
    gt_citation.cpp
    gt_wkt_srs.cpp
    tifvsi.cpp)
endif()

###############################################################################
# Source files specification

set(LIBPC_HEADERS_DIR ../include/libpc)

set(LIBPC_HPP
  ${LIBPC_HEADERS_DIR}/exceptions.hpp
  ${LIBPC_HEADERS_DIR}/export.hpp
  ${LIBPC_HEADERS_DIR}/libpc_config.hpp
  ${LIBPC_HEADERS_DIR}/Bounds.hpp
  ${LIBPC_HEADERS_DIR}/CacheFilter.hpp
  ${LIBPC_HEADERS_DIR}/Color.hpp
  ${LIBPC_HEADERS_DIR}/ColorFilter.hpp
  ${LIBPC_HEADERS_DIR}/CropFilter.hpp
  ${LIBPC_HEADERS_DIR}/Dimension.hpp
  ${LIBPC_HEADERS_DIR}/DimensionLayout.hpp
  ${LIBPC_HEADERS_DIR}/FauxReader.hpp 
  ${LIBPC_HEADERS_DIR}/FauxWriter.hpp
  ${LIBPC_HEADERS_DIR}/Filter.hpp
  ${LIBPC_HEADERS_DIR}/Header.hpp
  ${LIBPC_HEADERS_DIR}/Header.hpp
  ${LIBPC_HEADERS_DIR}/LasHeader.hpp
  ${LIBPC_HEADERS_DIR}/LasHeaderReader.hpp
  ${LIBPC_HEADERS_DIR}/LasHeaderWriter.hpp
  ${LIBPC_HEADERS_DIR}/LasReader.hpp
  ${LIBPC_HEADERS_DIR}/LasWriter.hpp
  ${LIBPC_HEADERS_DIR}/Metadata.hpp
  ${LIBPC_HEADERS_DIR}/MosaicFilter.hpp
  ${LIBPC_HEADERS_DIR}/Vector.hpp  
  ${LIBPC_HEADERS_DIR}/PointData.hpp  
  ${LIBPC_HEADERS_DIR}/Range.hpp
  ${LIBPC_HEADERS_DIR}/Producer.hpp
  ${LIBPC_HEADERS_DIR}/Schema.hpp
  ${LIBPC_HEADERS_DIR}/SchemaLayout.hpp
  ${LIBPC_HEADERS_DIR}/Signaller.hpp
  ${LIBPC_HEADERS_DIR}/SpatialReference.hpp
  ${LIBPC_HEADERS_DIR}/Stage.hpp
  ${LIBPC_HEADERS_DIR}/Utils.hpp
  ${LIBPC_HEADERS_DIR}/Consumer.hpp)

set (LIBPC_SOURCES "libpc_config.cpp" CACHE INTERNAL "sources to compile")
set (APPS_CPP_DEPENDENCIES "${LIBPC_LIB_NAME}" CACHE INTERNAL "libraries to link")

add_subdirectory(drivers)


set(LIBPC_CPP
    ${LIBPC_SOURCES}
  Bounds.cpp
  chipper.cpp
  CacheFilter.cpp
  Color.cpp
  ColorFilter.cpp
  CropFilter.cpp
  Dimension.cpp
  DimensionLayout.cpp
  FauxReader.cpp
  FauxWriter.cpp
  Filter.cpp
  Header.cpp
  LasHeader.cpp
  LasHeaderReader.cpp
  LasHeaderWriter.cpp
  LasReader.cpp
  LasWriter.cpp
  Metadata.cpp
  MosaicFilter.cpp
  Vector.cpp  
  PointData.cpp
  Producer.cpp
  Schema.cpp
  SchemaLayout.cpp
  Signaller.cpp
  SpatialReference.cpp
  Stage.cpp
  Utils.cpp
  Consumer.cpp)


# Group source files for IDE source explorers (e.g. Visual Studio)
source_group("CMake Files" FILES CMakeLists.txt)
source_group("Header Files" FILES ${LIBPC_HPP})
source_group("Source Files" FILES ${LIBPC_CPP})
if(GDAL_FOUND)
    source_group("Source Files\\gdal" FILES ${LIBPC_GDAL_CPP})
endif()

# Standard include directory of libPC library
include_directories(../include)

###############################################################################
# Targets settings

set(LIBPC_SOURCES
  ${LIBPC_HPP}
  ${LIBPC_CPP})

# NOTE:
# This hack is required to correctly link static into shared library.
# Such practice is not recommended as not portable, instead each library,
# static and shared should be built from sources separately.
#if(UNIX)
#  add_definitions("-fPIC")
#endif()

if(WIN32)
    add_definitions("-DLIBPC_DLL_EXPORT=1")
if (NOT WITH_STATIC_LASZIP)
    add_definitions("-DLASZIP_DLL_IMPORT=1")
endif()
endif()

add_library(${LIBPC_LIB_NAME} SHARED ${LIBPC_SOURCES})

target_link_libraries(${APPS_CPP_DEPENDENCIES}
  ${LIBPC_LIB_NAME}
  ${LIBLAS_LIBRARY}
  ${TIFF_LIBRARY}
  ${GEOTIFF_LIBRARY}
  ${GDAL_LIBRARY}
  ${LASZIP_LIBRARY}
  ${Boost_LIBRARIES})


set_target_properties(${LIBPC_LIB_NAME}
  PROPERTIES SOVERSION "${LIBPC_LIB_SOVERSION}" )

if (APPLE)
  set_target_properties(
    ${LIBPC_LIB_NAME}
    PROPERTIES
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()

###############################################################################
# Targets installation

install(TARGETS ${LIBPC_LIB_NAME} ${LIBPC_C_LIB_NAME}
  RUNTIME DESTINATION ${LIBPC_BIN_DIR}
  LIBRARY DESTINATION ${LIBPC_LIB_DIR}
  ARCHIVE DESTINATION ${LIBPC_LIB_DIR})

install(DIRECTORY ${LIBPC_HEADERS_DIR}
  DESTINATION ${LIBPC_INCLUDE_DIR}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
