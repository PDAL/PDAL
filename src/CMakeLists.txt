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
#
# Naming format:
#   LIBPC_CPP - all the .cpp files
#   LIBPC_HPP - all the .hpp files
#   LIBPC_<dirname>_CPP - all the .cpp files for the given subdir/namespace
#   ...

set(LIBPC_HEADERS_DIR ../include/libpc)
set(LIBPC_HPP "")
set(LIBPC_CPP "")

#
# base
#
set(LIBPC_BASE_HPP
  ${LIBPC_HEADERS_DIR}/exceptions.hpp
  ${LIBPC_HEADERS_DIR}/types.hpp
  ${LIBPC_HEADERS_DIR}/Bounds.hpp
  ${LIBPC_HEADERS_DIR}/Color.hpp
  ${LIBPC_HEADERS_DIR}/Dimension.hpp
  ${LIBPC_HEADERS_DIR}/DimensionLayout.hpp
  ${LIBPC_HEADERS_DIR}/Filter.hpp
  ${LIBPC_HEADERS_DIR}/FilterIterator.hpp
  ${LIBPC_HEADERS_DIR}/Iterator.hpp
  ${LIBPC_HEADERS_DIR}/MetadataRecord.hpp
  ${LIBPC_HEADERS_DIR}/Vector.hpp  
  ${LIBPC_HEADERS_DIR}/Options.hpp
  ${LIBPC_HEADERS_DIR}/PointBuffer.hpp  
  ${LIBPC_HEADERS_DIR}/PointBufferCache.hpp  
  ${LIBPC_HEADERS_DIR}/Range.hpp
  ${LIBPC_HEADERS_DIR}/Schema.hpp
  ${LIBPC_HEADERS_DIR}/SchemaLayout.hpp
  ${LIBPC_HEADERS_DIR}/Signaller.hpp
  ${LIBPC_HEADERS_DIR}/SpatialReference.hpp
  ${LIBPC_HEADERS_DIR}/Stage.hpp
  ${LIBPC_HEADERS_DIR}/Utils.hpp
  ${LIBPC_HEADERS_DIR}/Writer.hpp
)

set(LIBPC_BASE_CPP
  Bounds.cpp
  Color.cpp
  Dimension.cpp
  DimensionLayout.cpp
  Filter.cpp
  FilterIterator.cpp
  Iterator.cpp
  MetadataRecord.cpp
  Vector.cpp  
  PointBuffer.cpp
  PointBufferCache.cpp
  Options.cpp
  Schema.cpp
  SchemaLayout.cpp
  Signaller.cpp
  SpatialReference.cpp
  Stage.cpp
  Utils.cpp
  Writer.cpp
)

FOREACH(file ${LIBPC_BASE_HPP})
        SET(LIBPC_HPP "${LIBPC_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${LIBPC_BASE_CPP})
        SET(LIBPC_CPP "${LIBPC_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

#
# drivers/faux
#
set(LIBPC_DRIVERS_FAUX_HPP
  ${LIBPC_HEADERS_DIR}/drivers/faux/Iterator.hpp
  ${LIBPC_HEADERS_DIR}/drivers/faux/Reader.hpp
  ${LIBPC_HEADERS_DIR}/drivers/faux/Writer.hpp
)

set (LIBPC_DRIVERS_FAUX_CPP
  ./drivers/faux/Iterator.cpp
  ./drivers/faux/Reader.cpp
  ./drivers/faux/Writer.cpp 
)

FOREACH(file ${LIBPC_DRIVERS_FAUX_HPP})
        SET(LIBPC_HPP "${LIBPC_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${LIBPC_DRIVERS_FAUX_CPP})
        SET(LIBPC_CPP "${LIBPC_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

#
# drivers/las
#

set (LIBPC_DRIVERS_LAS_HPP
  ./drivers/las/LasHeaderReader.hpp
  ./drivers/las/LasHeaderWriter.hpp
  ./drivers/las/ZipPoint.hpp
  ${LIBPC_HEADERS_DIR}/drivers/las/Header.hpp
  ${LIBPC_HEADERS_DIR}/drivers/las/Iterator.hpp
  ${LIBPC_HEADERS_DIR}/drivers/las/Reader.hpp
  ${LIBPC_HEADERS_DIR}/drivers/las/ReaderBase.hpp
  ${LIBPC_HEADERS_DIR}/drivers/las/SummaryData.hpp
  ${LIBPC_HEADERS_DIR}/drivers/las/Support.hpp
  ${LIBPC_HEADERS_DIR}/drivers/las/Writer.hpp
  ${LIBPC_HEADERS_DIR}/drivers/las/VariableLengthRecord.hpp
)
 
set (LIBPC_DRIVERS_LAS_CPP
  ./drivers/las/Header.cpp
  ./drivers/las/Iterator.cpp
  ./drivers/las/LasHeaderReader.cpp
  ./drivers/las/LasHeaderWriter.cpp
  ./drivers/las/Reader.cpp
  ./drivers/las/SummaryData.cpp
  ./drivers/las/Support.cpp
  ./drivers/las/VariableLengthRecord.cpp
  ./drivers/las/Writer.cpp
  ./drivers/las/ZipPoint.cpp
)
  
FOREACH(file ${LIBPC_DRIVERS_LAS_HPP})
        SET(LIBPC_HPP "${LIBPC_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${LIBPC_DRIVERS_LAS_CPP})
        SET(LIBPC_CPP "${LIBPC_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

#
# drivers/liblas
#

set (LIBPC_DRIVERS_LIBLAS_HPP
  ${LIBPC_HEADERS_DIR}/drivers/liblas/Iterator.hpp
  ${LIBPC_HEADERS_DIR}/drivers/liblas/Reader.hpp
  ${LIBPC_HEADERS_DIR}/drivers/liblas/Writer.hpp
)

set (LIBPC_DRIVERS_LIBLAS_CPP 
  ./drivers/liblas/Iterator.cpp
  ./drivers/liblas/Reader.cpp
  ./drivers/liblas/Writer.cpp
)

FOREACH(file ${LIBPC_DRIVERS_LIBLAS_HPP})
        SET(LIBPC_HPP "${LIBPC_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${LIBPC_DRIVERS_LIBLAS_CPP})
        SET(LIBPC_CPP "${LIBPC_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

#
# drivers/oci
#
if (WITH_ORACLE)
set(LIBPC_DRIVERS_OCI_HPP
  ${LIBPC_HEADERS_DIR}/drivers/oci/Writer.hpp
  ${LIBPC_HEADERS_DIR}/drivers/oci/Reader.hpp
  ${LIBPC_HEADERS_DIR}/drivers/oci/Iterator.hpp
  ${LIBPC_HEADERS_DIR}/drivers/oci/oci_wrapper.h
  ${LIBPC_HEADERS_DIR}/drivers/oci/Common.hpp
  ${LIBPC_HEADERS_DIR}/drivers/oci/Schema.hpp
)

set (LIBPC_DRIVERS_OCI_CPP 
  ./drivers/oci/oci_wrapper.cpp
  ./drivers/oci/common.cpp
  ./drivers/oci/Reader.cpp
  ./drivers/oci/Iterator.cpp
  ./drivers/oci/Writer.cpp
  ./drivers/oci/Schema.cpp
)
   
FOREACH(file ${LIBPC_DRIVERS_OCI_HPP})
        SET(LIBPC_HPP "${LIBPC_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${LIBPC_DRIVERS_OCI_CPP})
        SET(LIBPC_CPP "${LIBPC_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)
endif()

#
# drivers/mrsid
#
if (MRSID_FOUND)
set(LIBPC_DRIVERS_MRSID_HPP
  ${LIBPC_HEADERS_DIR}/drivers/mrsid/Reader.hpp
)

set (LIBPC_DRIVERS_MRSID_CPP 
  ./drivers/mrsid/Reader.cpp
)
   
FOREACH(file ${LIBPC_DRIVERS_MRSID_HPP})
        SET(LIBPC_HPP "${LIBPC_HPP};${file}" CACHE INTERNAL "source files for drivers/mrsid")
ENDFOREACH(file)

FOREACH(file ${LIBPC_DRIVERS_MRSID_CPP})
        SET(LIBPC_CPP "${LIBPC_CPP};${file}" CACHE INTERNAL "source files for drivers/mrsid")
ENDFOREACH(file)
endif()

#
# filters
#

set(LIBPC_FILTERS_HPP
  ${LIBPC_HEADERS_DIR}/filters/CacheFilter.hpp
  ${LIBPC_HEADERS_DIR}/filters/CacheFilterIterator.hpp
  ${LIBPC_HEADERS_DIR}/filters/Chipper.hpp
  ${LIBPC_HEADERS_DIR}/filters/ChipperIterator.hpp
  ${LIBPC_HEADERS_DIR}/filters/ColorFilter.hpp
  ${LIBPC_HEADERS_DIR}/filters/ColorFilterIterator.hpp
  ${LIBPC_HEADERS_DIR}/filters/CropFilter.hpp
  ${LIBPC_HEADERS_DIR}/filters/CropFilterIterator.hpp
  ${LIBPC_HEADERS_DIR}/filters/DecimationFilter.hpp
  ${LIBPC_HEADERS_DIR}/filters/DecimationFilterIterator.hpp
  ${LIBPC_HEADERS_DIR}/filters/MosaicFilter.hpp
  ${LIBPC_HEADERS_DIR}/filters/MosaicFilterIterator.hpp
  ${LIBPC_HEADERS_DIR}/filters/ReprojectionFilter.hpp
  ${LIBPC_HEADERS_DIR}/filters/ReprojectionFilterIterator.hpp
)

set (LIBPC_FILTERS_CPP 
  ./filters/CacheFilter.cpp
  ./filters/CacheFilterIterator.cpp
  ./filters/Chipper.cpp
  ./filters/ChipperIterator.cpp
  ./filters/ColorFilter.cpp
  ./filters/ColorFilterIterator.cpp
  ./filters/CropFilter.cpp
  ./filters/CropFilterIterator.cpp
  ./filters/DecimationFilter.cpp
  ./filters/DecimationFilterIterator.cpp
  ./filters/MosaicFilter.cpp
  ./filters/MosaicFilterIterator.cpp
  ./filters/ReprojectionFilter.cpp
  ./filters/ReprojectionFilterIterator.cpp
)

FOREACH(file ${LIBPC_FILTERS_HPP})
        SET(LIBPC_HPP "${LIBPC_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${LIBPC_FILTERS_CPP})
        SET(LIBPC_CPP "${LIBPC_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

#
# config
#

set(LIBPC_CONFIG_HPP
  ${LIBPC_HEADERS_DIR}/export.hpp
  ${LIBPC_HEADERS_DIR}/libpc.hpp
  ${LIBPC_HEADERS_DIR}/libpc_config.hpp
  ${LIBPC_HEADERS_DIR}/libpc_defines.h
)

set (LIBPC_CONFIG_CPP 
  ./libpc_config.cpp
)

FOREACH(file ${LIBPC_CONFIG_HPP})
        SET(LIBPC_HPP "${LIBPC_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${LIBPC_CONFIG_CPP})
        SET(LIBPC_CPP "${LIBPC_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

#
# Group source files for IDE source explorers (e.g. Visual Studio)
#

source_group("CMake Files" FILES CMakeLists.txt)

source_group("Header Files" FILES ${LIBPC_BASE_HPP})
source_group("Header Files\\config" FILES ${LIBPC_CONFIG_HPP})
source_group("Header Files\\drivers\\faux" FILES ${LIBPC_DRIVERS_FAUX_HPP})
source_group("Header Files\\drivers\\las" FILES ${LIBPC_DRIVERS_LAS_HPP})
source_group("Header Files\\drivers\\liblas" FILES ${LIBPC_DRIVERS_LIBLAS_HPP})
source_group("Header Files\\drivers\\oci" FILES ${LIBPC_DRIVERS_OCI_HPP})
source_group("Header Files\\filters" FILES ${LIBPC_FILTERS_HPP})

source_group("Source Files" FILES ${LIBPC_BASE_CPP})
source_group("Source Files\\config" FILES ${LIBPC_CONFIG_CPP})
source_group("Source Files\\filter" FILES ${LIBPC_FILTERS_CPP})
source_group("Source Files\\drivers\\faux" FILES ${LIBPC_DRIVERS_FAUX_CPP})
source_group("Source Files\\drivers\\las" FILES ${LIBPC_DRIVERS_LAS_CPP})
source_group("Source Files\\drivers\\liblas" FILES ${LIBPC_DRIVERS_LIBLAS_CPP})
source_group("Source Files\\drivers\\oci" FILES ${LIBPC_DRIVERS_OCI_CPP})

#
# rest of stuff
#

if(GDAL_FOUND)
    source_group("Source Files\\gdal" FILES ${LIBPC_GDAL_CPP})
endif()

# Standard include directory of libPC library
include_directories(../include)

set (APPS_CPP_DEPENDENCIES "${LIBPC_LIB_NAME}" CACHE INTERNAL "libraries to link")

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
  ${ORACLE_LIBRARY}
  ${MRSID_LIBRARY}
  ${GEOTIFF_LIBRARY}
  ${GDAL_LIBRARY}
  ${LASZIP_LIBRARY}
  ${Boost_LIBRARIES}
  ${LIBXML2_LIBRARIES})


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
