###############################################################################
#
# src/CMakeLists.txt controls building of PDAL library
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
###############################################################################

# Collect dependencies configuration
if(GDAL_FOUND)
  set(PDAL_GDAL_CPP
    gt_citation.cpp
    gt_wkt_srs.cpp
    tifvsi.cpp)
endif()


###############################################################################
# Source files specification
#
# Naming format:
#   PDAL_CPP - all the .cpp files
#   PDAL_HPP - all the .hpp files
#   PDAL_<dirname>_CPP - all the .cpp files for the given subdir/namespace
#   ...

set(PDAL_HEADERS_DIR ../include/pdal)
set(PDAL_HPP "")
set(PDAL_CPP "")

#
# base
#
set(PDAL_BASE_HPP
  ${PDAL_HEADERS_DIR}/exceptions.hpp
  ${PDAL_HEADERS_DIR}/types.hpp
  ${PDAL_HEADERS_DIR}/Bounds.hpp
  ${PDAL_HEADERS_DIR}/Color.hpp
  ${PDAL_HEADERS_DIR}/Dimension.hpp
  ${PDAL_HEADERS_DIR}/DimensionLayout.hpp
  ${PDAL_HEADERS_DIR}/Filter.hpp
  ${PDAL_HEADERS_DIR}/FilterIterator.hpp
  ${PDAL_HEADERS_DIR}/Iterator.hpp
  ${PDAL_HEADERS_DIR}/MetadataRecord.hpp
  ${PDAL_HEADERS_DIR}/Vector.hpp  
  ${PDAL_HEADERS_DIR}/Options.hpp
  ${PDAL_HEADERS_DIR}/PointBuffer.hpp  
  ${PDAL_HEADERS_DIR}/PointBufferCache.hpp  
  ${PDAL_HEADERS_DIR}/Range.hpp
  ${PDAL_HEADERS_DIR}/Schema.hpp
  ${PDAL_HEADERS_DIR}/SchemaLayout.hpp
  ${PDAL_HEADERS_DIR}/Signaller.hpp
  ${PDAL_HEADERS_DIR}/SpatialReference.hpp
  ${PDAL_HEADERS_DIR}/Stage.hpp
  ${PDAL_HEADERS_DIR}/Utils.hpp
  ${PDAL_HEADERS_DIR}/Writer.hpp
  ${PDAL_HEADERS_DIR}/XMLSchema.hpp
)

if (PDAL_HAVE_LIBXML2)
  SET (PDAL_XML_SRC XMLSchema.cpp)
endif()

set(PDAL_BASE_CPP
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
  Range.cpp
  Schema.cpp
  SchemaLayout.cpp
  Signaller.cpp
  SpatialReference.cpp
  Stage.cpp
  Utils.cpp
  Writer.cpp
  ${PDAL_XML_SRC}
)


FOREACH(file ${PDAL_BASE_HPP})
        SET(PDAL_HPP "${PDAL_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${PDAL_BASE_CPP})
        SET(PDAL_CPP "${PDAL_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

#
# drivers/faux
#
set(PDAL_DRIVERS_FAUX_HPP
  ${PDAL_HEADERS_DIR}/drivers/faux/Iterator.hpp
  ${PDAL_HEADERS_DIR}/drivers/faux/Reader.hpp
  ${PDAL_HEADERS_DIR}/drivers/faux/Writer.hpp
)

set (PDAL_DRIVERS_FAUX_CPP
  ./drivers/faux/Iterator.cpp
  ./drivers/faux/Reader.cpp
  ./drivers/faux/Writer.cpp 
)

FOREACH(file ${PDAL_DRIVERS_FAUX_HPP})
        SET(PDAL_HPP "${PDAL_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${PDAL_DRIVERS_FAUX_CPP})
        SET(PDAL_CPP "${PDAL_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

#
# drivers/las
#

set (PDAL_DRIVERS_LAS_INCLUDE_DIR ${PDAL_HEADERS_DIR}/drivers/las)
set (PDAL_DRIVERS_LAS_SRC_DIR ./drivers/las)

set (PDAL_DRIVERS_LAS_HPP
  ${PDAL_DRIVERS_LAS_SRC_DIR}/GeotiffSupport.hpp
  ${PDAL_DRIVERS_LAS_SRC_DIR}/LasHeaderReader.hpp
  ${PDAL_DRIVERS_LAS_SRC_DIR}/LasHeaderWriter.hpp
  ${PDAL_DRIVERS_LAS_SRC_DIR}/ZipPoint.hpp
  ${PDAL_DRIVERS_LAS_INCLUDE_DIR}/Header.hpp
  ${PDAL_DRIVERS_LAS_INCLUDE_DIR}/Iterator.hpp
  ${PDAL_DRIVERS_LAS_INCLUDE_DIR}/Reader.hpp
  ${PDAL_DRIVERS_LAS_INCLUDE_DIR}/ReaderBase.hpp
  ${PDAL_DRIVERS_LAS_INCLUDE_DIR}/SummaryData.hpp
  ${PDAL_DRIVERS_LAS_INCLUDE_DIR}/Support.hpp
  ${PDAL_DRIVERS_LAS_INCLUDE_DIR}/Writer.hpp
  ${PDAL_DRIVERS_LAS_INCLUDE_DIR}/VariableLengthRecord.hpp
)

if (GEOTIFF_FOUND)
    if (PDAL_HAVE_GDAL)
    set (PDAL_DRIVERS_LAS_GTIFF ${PDAL_DRIVERS_LAS_SRC_DIR}/GeotiffSupport.cpp)
    endif()
endif()

set (PDAL_DRIVERS_LAS_CPP
  ${PDAL_DRIVERS_LAS_GTIFF}
  ${PDAL_DRIVERS_LAS_SRC_DIR}/Header.cpp
  ${PDAL_DRIVERS_LAS_SRC_DIR}//Iterator.cpp
  ${PDAL_DRIVERS_LAS_SRC_DIR}/LasHeaderReader.cpp
  ${PDAL_DRIVERS_LAS_SRC_DIR}/LasHeaderWriter.cpp
  ${PDAL_DRIVERS_LAS_SRC_DIR}/Reader.cpp
  ${PDAL_DRIVERS_LAS_SRC_DIR}/SummaryData.cpp
  ${PDAL_DRIVERS_LAS_SRC_DIR}/Support.cpp
  ${PDAL_DRIVERS_LAS_SRC_DIR}/VariableLengthRecord.cpp
  ${PDAL_DRIVERS_LAS_SRC_DIR}/Writer.cpp
  ${PDAL_DRIVERS_LAS_SRC_DIR}/ZipPoint.cpp
)
  
FOREACH(file ${PDAL_DRIVERS_LAS_HPP})
        SET(PDAL_HPP "${PDAL_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${PDAL_DRIVERS_LAS_CPP})
        SET(PDAL_CPP "${PDAL_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

#
# drivers/liblas
#

set (PDAL_DRIVERS_LIBLAS_HPP
  ${PDAL_HEADERS_DIR}/drivers/liblas/Iterator.hpp
  ${PDAL_HEADERS_DIR}/drivers/liblas/Reader.hpp
  ${PDAL_HEADERS_DIR}/drivers/liblas/Writer.hpp
)

set (PDAL_DRIVERS_LIBLAS_CPP 
  ./drivers/liblas/Iterator.cpp
  ./drivers/liblas/Reader.cpp
  ./drivers/liblas/Writer.cpp
)

FOREACH(file ${PDAL_DRIVERS_LIBLAS_HPP})
        SET(PDAL_HPP "${PDAL_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${PDAL_DRIVERS_LIBLAS_CPP})
        SET(PDAL_CPP "${PDAL_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

#
# drivers/oci
#
if (WITH_ORACLE)
set(PDAL_DRIVERS_OCI_HPP
  ${PDAL_HEADERS_DIR}/drivers/oci/Writer.hpp
  ${PDAL_HEADERS_DIR}/drivers/oci/Reader.hpp
  ${PDAL_HEADERS_DIR}/drivers/oci/Iterator.hpp
  ${PDAL_HEADERS_DIR}/drivers/oci/oci_wrapper.h
  ${PDAL_HEADERS_DIR}/drivers/oci/Common.hpp
)

set (PDAL_DRIVERS_OCI_CPP 
  ./drivers/oci/oci_wrapper.cpp
  ./drivers/oci/common.cpp
  ./drivers/oci/Reader.cpp
  ./drivers/oci/Iterator.cpp
  ./drivers/oci/Writer.cpp
)
   
FOREACH(file ${PDAL_DRIVERS_OCI_HPP})
        SET(PDAL_HPP "${PDAL_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${PDAL_DRIVERS_OCI_CPP})
        SET(PDAL_CPP "${PDAL_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)
endif()

#
# drivers/mrsid
#
if (MRSID_FOUND)
set(PDAL_DRIVERS_MRSID_HPP
  ${PDAL_HEADERS_DIR}/drivers/mrsid/Reader.hpp
)

set (PDAL_DRIVERS_MRSID_CPP 
  ./drivers/mrsid/Reader.cpp
)
   
FOREACH(file ${PDAL_DRIVERS_MRSID_HPP})
        SET(PDAL_HPP "${PDAL_HPP};${file}" CACHE INTERNAL "source files for drivers/mrsid")
ENDFOREACH(file)

FOREACH(file ${PDAL_DRIVERS_MRSID_CPP})
        SET(PDAL_CPP "${PDAL_CPP};${file}" CACHE INTERNAL "source files for drivers/mrsid")
ENDFOREACH(file)
endif()


#
# drivers/qfit
#
set(PDAL_DRIVERS_QFIT_HPP
  ${PDAL_HEADERS_DIR}/drivers/qfit/Iterator.hpp
  ${PDAL_HEADERS_DIR}/drivers/qfit/Reader.hpp
)

set (PDAL_DRIVERS_QFIT_CPP 
  ./drivers/qfit/Iterator.cpp
  ./drivers/qfit/Reader.cpp
)
   
FOREACH(file ${PDAL_DRIVERS_QFIT_HPP})
        SET(PDAL_HPP "${PDAL_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${PDAL_DRIVERS_QFIT_CPP})
        SET(PDAL_CPP "${PDAL_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

#
# drivers/terrasolid
#
set(PDAL_DRIVERS_TERRASOLID_HPP
  ${PDAL_HEADERS_DIR}/drivers/terrasolid/Iterator.hpp
  ${PDAL_HEADERS_DIR}/drivers/terrasolid/Reader.hpp
)

set (PDAL_DRIVERS_TERRASOLID_CPP 
  ./drivers/terrasolid/Iterator.cpp
  ./drivers/terrasolid/Reader.cpp
)
   
FOREACH(file ${PDAL_DRIVERS_TERRASOLID_HPP})
        SET(PDAL_HPP "${PDAL_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${PDAL_DRIVERS_TERRASOLID_CPP})
        SET(PDAL_CPP "${PDAL_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

#
# filters
#

set(PDAL_FILTERS_HPP
  ${PDAL_HEADERS_DIR}/filters/ByteSwapFilter.hpp
  ${PDAL_HEADERS_DIR}/filters/ByteSwapFilterIterator.hpp
  ${PDAL_HEADERS_DIR}/filters/CacheFilter.hpp
  ${PDAL_HEADERS_DIR}/filters/CacheFilterIterator.hpp
  ${PDAL_HEADERS_DIR}/filters/Chipper.hpp
  ${PDAL_HEADERS_DIR}/filters/ChipperIterator.hpp
  ${PDAL_HEADERS_DIR}/filters/ColorFilter.hpp
  ${PDAL_HEADERS_DIR}/filters/ColorFilterIterator.hpp
  ${PDAL_HEADERS_DIR}/filters/CropFilter.hpp
  ${PDAL_HEADERS_DIR}/filters/CropFilterIterator.hpp
  ${PDAL_HEADERS_DIR}/filters/DecimationFilter.hpp
  ${PDAL_HEADERS_DIR}/filters/DecimationFilterIterator.hpp
  ${PDAL_HEADERS_DIR}/filters/MosaicFilter.hpp
  ${PDAL_HEADERS_DIR}/filters/MosaicFilterIterator.hpp
  ${PDAL_HEADERS_DIR}/filters/ReprojectionFilter.hpp
  ${PDAL_HEADERS_DIR}/filters/ReprojectionFilterIterator.hpp
  ${PDAL_HEADERS_DIR}/filters/ScalingFilter.hpp
  ${PDAL_HEADERS_DIR}/filters/ScalingFilterIterator.hpp
)

set (PDAL_FILTERS_CPP 
  ./filters/ByteSwapFilter.cpp
  ./filters/ByteSwapFilterIterator.cpp
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
  ./filters/ScalingFilter.cpp
  ./filters/ScalingFilterIterator.cpp
)

FOREACH(file ${PDAL_FILTERS_HPP})
        SET(PDAL_HPP "${PDAL_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${PDAL_FILTERS_CPP})
        SET(PDAL_CPP "${PDAL_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

#
# config
#

set(PDAL_CONFIG_HPP
  ${PDAL_HEADERS_DIR}/export.hpp
  ${PDAL_HEADERS_DIR}/pdal.hpp
  ${PDAL_HEADERS_DIR}/pdal_config.hpp
  ${PDAL_HEADERS_DIR}/pdal_defines.h
)

set (PDAL_CONFIG_CPP 
  ./pdal_config.cpp
)

FOREACH(file ${PDAL_CONFIG_HPP})
        SET(PDAL_HPP "${PDAL_HPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

FOREACH(file ${PDAL_CONFIG_CPP})
        SET(PDAL_CPP "${PDAL_CPP};${file}" CACHE INTERNAL "source files for foo")
ENDFOREACH(file)

#
# Group source files for IDE source explorers (e.g. Visual Studio)
#

source_group("CMake Files" FILES CMakeLists.txt)

source_group("Header Files" FILES ${PDAL_BASE_HPP})
source_group("Header Files\\config" FILES ${PDAL_CONFIG_HPP})
source_group("Header Files\\drivers\\faux" FILES ${PDAL_DRIVERS_FAUX_HPP})
source_group("Header Files\\drivers\\las" FILES ${PDAL_DRIVERS_LAS_HPP})
source_group("Header Files\\drivers\\liblas" FILES ${PDAL_DRIVERS_LIBLAS_HPP})
source_group("Header Files\\drivers\\oci" FILES ${PDAL_DRIVERS_OCI_HPP})
source_group("Header Files\\drivers\\qfit" FILES ${PDAL_DRIVERS_QFIT_HPP})
source_group("Header Files\\drivers\\terrasolid" FILES ${PDAL_DRIVERS_TERRASOLID_HPP})
source_group("Header Files\\filters" FILES ${PDAL_FILTERS_HPP})

source_group("Source Files" FILES ${PDAL_BASE_CPP})
source_group("Source Files\\config" FILES ${PDAL_CONFIG_CPP})
source_group("Source Files\\drivers\\faux" FILES ${PDAL_DRIVERS_FAUX_CPP})
source_group("Source Files\\drivers\\las" FILES ${PDAL_DRIVERS_LAS_CPP})
source_group("Source Files\\drivers\\liblas" FILES ${PDAL_DRIVERS_LIBLAS_CPP})
source_group("Source Files\\drivers\\oci" FILES ${PDAL_DRIVERS_OCI_CPP})
source_group("Source Files\\drivers\\qfit" FILES ${PDAL_DRIVERS_QFIT_CPP})
source_group("Source Files\\drivers\\terrasolid" FILES ${PDAL_DRIVERS_TERRASOLID_CPP})
source_group("Source Files\\filters" FILES ${PDAL_FILTERS_CPP})

#
# rest of stuff
#

if(GDAL_FOUND)
    source_group("Source Files\\gdal" FILES ${PDAL_GDAL_CPP})
endif()

# Standard include directory of PDAL library
include_directories(../include)

set (APPS_CPP_DEPENDENCIES "${PDAL_LIB_NAME}" CACHE INTERNAL "libraries to link")

###############################################################################
# Targets settings

set(PDAL_SOURCES
  ${PDAL_HPP}
  ${PDAL_CPP})

# NOTE:
# This hack is required to correctly link static into shared library.
# Such practice is not recommended as not portable, instead each library,
# static and shared should be built from sources separately.
#if(UNIX)
#  add_definitions("-fPIC")
#endif()

if(WIN32)
    add_definitions("-DPDAL_DLL_EXPORT=1")
if (NOT WITH_STATIC_LASZIP)
    add_definitions("-DLASZIP_DLL_IMPORT=1")
endif()
endif()

add_library(${PDAL_LIB_NAME} SHARED ${PDAL_SOURCES})

target_link_libraries(${APPS_CPP_DEPENDENCIES}
  ${PDAL_LIB_NAME}
  ${LIBLAS_LIBRARY}
  ${TIFF_LIBRARY}
  ${ORACLE_LIBRARY}
  ${MRSID_LIBRARY}
  ${GEOTIFF_LIBRARY}
  ${GDAL_LIBRARY}
  ${LASZIP_LIBRARY}
  ${Boost_LIBRARIES}
  ${LIBXML2_LIBRARIES})


set_target_properties(${PDAL_LIB_NAME}
  PROPERTIES SOVERSION "${PDAL_LIB_SOVERSION}" )

if (APPLE)
  set_target_properties(
    ${PDAL_LIB_NAME}
    PROPERTIES
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()

###############################################################################
# Targets installation

install(TARGETS ${PDAL_LIB_NAME} ${PDAL_C_LIB_NAME}
  RUNTIME DESTINATION ${PDAL_BIN_DIR}
  LIBRARY DESTINATION ${PDAL_LIB_DIR}
  ARCHIVE DESTINATION ${PDAL_LIB_DIR})

install(DIRECTORY ${PDAL_HEADERS_DIR}
  DESTINATION ${PDAL_INCLUDE_DIR}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
