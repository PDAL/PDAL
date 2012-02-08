###############################################################################
#
# src/CMakeLists.txt controls building of PDAL library
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
###############################################################################

###############################################################################
# Source files specification
#
# Naming format:
#   PDAL_CPP - all the .cpp files
#   PDAL_HPP - all the .hpp files
#   PDAL_<dirname>_CPP - all the .cpp files for the given subdir/namespace
#   ...

set(PDAL_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/pdal)
set(PDAL_HPP "")
set(PDAL_CPP "")

#
# base
#
set(PDAL_BASE_HPP
  ${PDAL_HEADERS_DIR}/pdal_error.hpp
  ${PDAL_HEADERS_DIR}/types.hpp
  ${PDAL_HEADERS_DIR}/Bounds.hpp
  ${PDAL_HEADERS_DIR}/Dimension.hpp
  ${PDAL_HEADERS_DIR}/FileUtils.hpp
  ${PDAL_HEADERS_DIR}/Filter.hpp
  ${PDAL_HEADERS_DIR}/FilterIterator.hpp
  ${PDAL_HEADERS_DIR}/GDALUtils.hpp
  ${PDAL_HEADERS_DIR}/Log.hpp
  ${PDAL_HEADERS_DIR}/MetadataRecord.hpp
  ${PDAL_HEADERS_DIR}/MultiFilter.hpp
  ${PDAL_HEADERS_DIR}/MultiFilterIterator.hpp
  ${PDAL_HEADERS_DIR}/Options.hpp
  ${PDAL_HEADERS_DIR}/PipelineManager.hpp
  ${PDAL_HEADERS_DIR}/PipelineReader.hpp
  ${PDAL_HEADERS_DIR}/PipelineWriter.hpp
  ${PDAL_HEADERS_DIR}/PointBuffer.hpp  
  ${PDAL_HEADERS_DIR}/Range.hpp
  ${PDAL_HEADERS_DIR}/Reader.hpp
  ${PDAL_HEADERS_DIR}/ReaderIterator.hpp
  ${PDAL_HEADERS_DIR}/Schema.hpp
  ${PDAL_HEADERS_DIR}/SpatialReference.hpp
  ${PDAL_HEADERS_DIR}/Stage.hpp
  ${PDAL_HEADERS_DIR}/StageBase.hpp
  ${PDAL_HEADERS_DIR}/StageFactory.hpp
  ${PDAL_HEADERS_DIR}/StageIterator.hpp
  ${PDAL_HEADERS_DIR}/StreamManager.hpp
  ${PDAL_HEADERS_DIR}/UserCallback.hpp
  ${PDAL_HEADERS_DIR}/Utils.hpp
  ${PDAL_HEADERS_DIR}/Vector.hpp  
  ${PDAL_HEADERS_DIR}/Writer.hpp
  ${PDAL_HEADERS_DIR}/XMLSchema.hpp
)

if (PDAL_HAVE_LIBXML2)
  SET (PDAL_XML_SRC XMLSchema.cpp)
endif()

set(PDAL_BASE_CPP
  Bounds.cpp
  Dimension.cpp
  FileUtils.cpp
  Filter.cpp
  FilterIterator.cpp
  Log.cpp
  MetadataRecord.cpp
  MultiFilter.cpp
  MultiFilterIterator.cpp
  Options.cpp
  PipelineManager.cpp
  PipelineReader.cpp
  PipelineWriter.cpp
  PointBuffer.cpp
  Range.cpp
  Reader.cpp
  ReaderIterator.cpp
  Schema.cpp
  SpatialReference.cpp
  Stage.cpp
  StageBase.cpp
  StageFactory.cpp
  StageIterator.cpp
  StreamManager.cpp
  UserCallback.cpp
  Utils.cpp
  Vector.cpp  
  Writer.cpp
  ${PDAL_XML_SRC}
)

if (GDAL_FOUND)
    set (PDAL_BASE_CPP  GDALUtils.cpp ${PDAL_BASE_CPP})
endif(GDAL_FOUND)

list (APPEND PDAL_CPP ${PDAL_BASE_CPP} )
list (APPEND PDAL_HPP ${PDAL_BASE_HPP} )

#
# drivers/faux
#
set(PDAL_FAUX_PATH drivers/faux)
set(PDAL_FAUX_HEADERS ${PDAL_HEADERS_DIR}/${PDAL_FAUX_PATH})
set(PDAL_FAUX_SRC ${PROJECT_SOURCE_DIR}/src/${PDAL_FAUX_PATH})
set(PDAL_DRIVERS_FAUX_HPP 
  ${PDAL_FAUX_HEADERS}/Reader.hpp
  ${PDAL_FAUX_HEADERS}/Writer.hpp
)

set (PDAL_DRIVERS_FAUX_CPP
  ${PDAL_FAUX_SRC}/Reader.cpp
  ${PDAL_FAUX_SRC}/Writer.cpp 
)
list (APPEND PDAL_CPP ${PDAL_DRIVERS_FAUX_CPP} )
list (APPEND PDAL_HPP ${PDAL_DRIVERS_FAUX_HPP} )


#
# drivers/las
#
set(PDAL_LAS_PATH drivers/las)
set(PDAL_LAS_HEADERS ${PDAL_HEADERS_DIR}/${PDAL_LAS_PATH})
set(PDAL_LAS_SRC ${PROJECT_SOURCE_DIR}/src/${PDAL_LAS_PATH})
set (PDAL_DRIVERS_LAS_HPP
  ${PDAL_LAS_SRC}/GeotiffSupport.hpp
  ${PDAL_LAS_SRC}/LasHeaderReader.hpp
  ${PDAL_LAS_SRC}/LasHeaderWriter.hpp
  ${PDAL_LAS_SRC}/ZipPoint.hpp
  ${PDAL_LAS_HEADERS}/Header.hpp
  ${PDAL_LAS_HEADERS}/Reader.hpp
  ${PDAL_LAS_HEADERS}/ReaderBase.hpp
  ${PDAL_LAS_HEADERS}/SummaryData.hpp
  ${PDAL_LAS_HEADERS}/Support.hpp
  ${PDAL_LAS_HEADERS}/Writer.hpp
  ${PDAL_LAS_HEADERS}/VariableLengthRecord.hpp
)

if ((GEOTIFF_FOUND) AND (GDAL_FOUND))
    set (PDAL_DRIVERS_LAS_GTIFF ${PDAL_LAS_SRC}/GeotiffSupport.cpp)
endif()

if (LASZIP_FOUND)
    set(PDAL_DRIVERS_LAS_LASZIP   ${PDAL_LAS_SRC}/ZipPoint.cpp)
endif()

set (PDAL_DRIVERS_LAS_CPP
  ${PDAL_DRIVERS_LAS_GTIFF}
  ${PDAL_DRIVERS_LAS_LASZIP}
  ${PDAL_LAS_SRC}/Header.cpp
  ${PDAL_LAS_SRC}/LasHeaderReader.cpp
  ${PDAL_LAS_SRC}/LasHeaderWriter.cpp
  ${PDAL_LAS_SRC}/Reader.cpp
  ${PDAL_LAS_SRC}/SummaryData.cpp
  ${PDAL_LAS_SRC}/Support.cpp
  ${PDAL_LAS_SRC}/VariableLengthRecord.cpp
  ${PDAL_LAS_SRC}/Writer.cpp
)
list (APPEND PDAL_CPP ${PDAL_DRIVERS_LAS_CPP} )
list (APPEND PDAL_HPP ${PDAL_DRIVERS_LAS_HPP} )

#
# drivers/mrsid
#
set(PDAL_MRSID_PATH drivers/mrsid)
set(PDAL_MRSID_HEADERS ${PDAL_HEADERS_DIR}/${PDAL_MRSID_PATH})
set(PDAL_MRSID_SRC ${PROJECT_SOURCE_DIR}/src/${PDAL_MRSID_PATH})

set(PDAL_DRIVERS_MRSID_HPP
  ${PDAL_MRSID_HEADERS}/Reader.hpp
)

set (PDAL_DRIVERS_MRSID_CPP 
  ${PDAL_MRSID_SRC}/Reader.cpp
)

if (MRSID_FOUND)
    list (APPEND PDAL_CPP ${PDAL_DRIVERS_MRSID_CPP} )
    list (APPEND PDAL_HPP ${PDAL_DRIVERS_MRSID_HPP} )
endif()


#
# drivers/oci
#

set(PDAL_ORACLE_PATH drivers/oci)
set(PDAL_ORACLE_HEADERS ${PDAL_HEADERS_DIR}/${PDAL_ORACLE_PATH})
set(PDAL_ORACLE_SRC ${PROJECT_SOURCE_DIR}/src/${PDAL_ORACLE_PATH})

set(PDAL_DRIVERS_OCI_HPP
  ${PDAL_ORACLE_HEADERS}/Writer.hpp
  ${PDAL_ORACLE_HEADERS}/Reader.hpp
  ${PDAL_ORACLE_HEADERS}/oci_wrapper.h
  ${PDAL_ORACLE_HEADERS}/common.hpp
)

set (PDAL_DRIVERS_OCI_CPP 
  ${PDAL_ORACLE_SRC}/oci_wrapper.cpp
  ${PDAL_ORACLE_SRC}/common.cpp
  ${PDAL_ORACLE_SRC}/Reader.cpp
  ${PDAL_ORACLE_SRC}/Writer.cpp
)
   
if (ORACLE_FOUND)
    list (APPEND PDAL_CPP ${PDAL_DRIVERS_OCI_CPP} )
    list (APPEND PDAL_HPP ${PDAL_DRIVERS_OCI_HPP} )
endif()


#
# drivers/pipeline
#

set(PDAL_PIPELINE_PATH drivers/pipeline)
set(PDAL_PIPELINE_HEADERS ${PDAL_HEADERS_DIR}/${PDAL_PIPELINE_PATH})
set(PDAL_PIPELINE_SRC ${PROJECT_SOURCE_DIR}/src/${PDAL_PIPELINE_PATH})

set (PDAL_DRIVERS_PIPELINE_HPP
  ${PDAL_PIPELINE_HEADERS}/Reader.hpp
)

set (PDAL_DRIVERS_PIPELINE_CPP 
  ${PDAL_PIPELINE_SRC}/Reader.cpp
)

list (APPEND PDAL_CPP ${PDAL_DRIVERS_PIPELINE_CPP} )
list (APPEND PDAL_HPP ${PDAL_DRIVERS_PIPELINE_HPP} )


#
# drivers/qfit
#
set(PDAL_QFIT_PATH drivers/qfit)
set(PDAL_QFIT_HEADERS ${PDAL_HEADERS_DIR}/${PDAL_QFIT_PATH})
set(PDAL_QFIT_SRC ${PROJECT_SOURCE_DIR}/src/${PDAL_QFIT_PATH})

set(PDAL_DRIVERS_QFIT_HPP
  ${PDAL_QFIT_HEADERS}/Reader.hpp
)

set (PDAL_DRIVERS_QFIT_CPP 
  ${PDAL_QFIT_SRC}/Reader.cpp
)

list (APPEND PDAL_CPP ${PDAL_DRIVERS_QFIT_CPP} )
list (APPEND PDAL_HPP ${PDAL_DRIVERS_QFIT_HPP} )

#
# drivers/terrasolid
#
set(PDAL_TERRASOLID_PATH drivers/terrasolid)
set(PDAL_TERRASOLID_HEADERS ${PDAL_HEADERS_DIR}/${PDAL_TERRASOLID_PATH})
set(PDAL_TERRASOLID_SRC ${PROJECT_SOURCE_DIR}/src/${PDAL_TERRASOLID_PATH})

set(PDAL_DRIVERS_TERRASOLID_HPP
  ${PDAL_TERRASOLID_HEADERS}/Reader.hpp
)

set (PDAL_DRIVERS_TERRASOLID_CPP 
  ${PDAL_TERRASOLID_SRC}/Reader.cpp
)

list (APPEND PDAL_CPP ${PDAL_DRIVERS_TERRASOLID_CPP} )
list (APPEND PDAL_HPP ${PDAL_DRIVERS_TERRASOLID_HPP} )

#
# drivers/p2g
#

set(PDAL_P2G_PATH drivers/p2g)
set(PDAL_P2G_HEADERS ${PDAL_HEADERS_DIR}/${PDAL_P2G_PATH})
set(PDAL_P2G_SRC ${PROJECT_SOURCE_DIR}/src/${PDAL_P2G_PATH})

set(PDAL_DRIVERS_P2G_HPP
  ${PDAL_P2G_HEADERS}/Writer.hpp
)

set (PDAL_DRIVERS_P2G_CPP 
  ${PDAL_P2G_SRC}/Writer.cpp
)
   
if (P2G_FOUND)
    list (APPEND PDAL_CPP ${PDAL_DRIVERS_P2G_CPP} )
    list (APPEND PDAL_HPP ${PDAL_DRIVERS_P2G_HPP} )
endif()

#
# drivers/text
#
set(PDAL_TEXT_PATH drivers/text)
set(PDAL_TEXT_HEADERS ${PDAL_HEADERS_DIR}/${PDAL_TEXT_PATH})
set(PDAL_TEXT_SRC ${PROJECT_SOURCE_DIR}/src/${PDAL_TEXT_PATH})

set(PDAL_DRIVERS_TEXT_HPP
  ${PDAL_TEXT_HEADERS}/Writer.hpp
)

set (PDAL_DRIVERS_TEXT_CPP 
  ${PDAL_TEXT_SRC}/Writer.cpp
)

list (APPEND PDAL_CPP ${PDAL_DRIVERS_TEXT_CPP} )
list (APPEND PDAL_HPP ${PDAL_DRIVERS_TEXT_HPP} )

#
# filters
#

set(PDAL_FILTERS_PATH filters)
set(PDAL_FILTERS_HEADERS ${PDAL_HEADERS_DIR}/${PDAL_FILTERS_PATH})
set(PDAL_FILTERS_SRC ${PROJECT_SOURCE_DIR}/src/${PDAL_FILTERS_PATH})

set(PDAL_FILTERS_HPP
  ${PDAL_FILTERS_HEADERS}/ByteSwap.hpp
  ${PDAL_FILTERS_HEADERS}/Cache.hpp
  ${PDAL_FILTERS_HEADERS}/Chipper.hpp
  ${PDAL_FILTERS_HEADERS}/Color.hpp
  ${PDAL_FILTERS_HEADERS}/Colorization.hpp
  ${PDAL_FILTERS_HEADERS}/Crop.hpp
  ${PDAL_FILTERS_HEADERS}/Decimation.hpp
  ${PDAL_FILTERS_HEADERS}/InPlaceReprojection.hpp
  ${PDAL_FILTERS_HEADERS}/Mosaic.hpp
  ${PDAL_FILTERS_HEADERS}/PointBufferCache.hpp
  ${PDAL_FILTERS_HEADERS}/Reprojection.hpp
  ${PDAL_FILTERS_HEADERS}/Scaling.hpp
  ${PDAL_FILTERS_HEADERS}/Selector.hpp
  ${PDAL_FILTERS_HEADERS}/Stats.hpp
)

set (PDAL_FILTERS_CPP 
  ${PDAL_FILTERS_SRC}/ByteSwap.cpp
  ${PDAL_FILTERS_SRC}/Cache.cpp
  ${PDAL_FILTERS_SRC}/Chipper.cpp
  ${PDAL_FILTERS_SRC}/Color.cpp
  ${PDAL_FILTERS_SRC}/Colorization.cpp
  ${PDAL_FILTERS_SRC}/Crop.cpp
  ${PDAL_FILTERS_SRC}/Decimation.cpp
  ${PDAL_FILTERS_SRC}/InPlaceReprojection.cpp
  ${PDAL_FILTERS_SRC}/Mosaic.cpp
  ${PDAL_FILTERS_SRC}/PointBufferCache.cpp
  ${PDAL_FILTERS_SRC}/Reprojection.cpp
  ${PDAL_FILTERS_SRC}/Scaling.cpp
  ${PDAL_FILTERS_SRC}/Selector.cpp
  ${PDAL_FILTERS_SRC}/Stats.cpp
)

list (APPEND PDAL_CPP ${PDAL_FILTERS_CPP} )
list (APPEND PDAL_HPP ${PDAL_FILTERS_HPP} )


#
# plag parser
#

set(PDAL_PLANG_PATH plang)
set(PDAL_PLANG_HEADERS ${PDAL_HEADERS_DIR}/${PDAL_PLANG_PATH})
set(PDAL_PLANG_SRC ${PROJECT_SOURCE_DIR}/src/${PDAL_PLANG_PATH})

set(PDAL_PLANG_HPP
  ${PDAL_PLANG_HEADERS}/AstNode.hpp
  ${PDAL_PLANG_HEADERS}/AstUtils.hpp
  ${PDAL_PLANG_HEADERS}/Parser.hpp
  ${PDAL_PLANG_HEADERS}/Program.hpp
  ${PDAL_PLANG_HEADERS}/SymbolTable.hpp
  ${PDAL_FILTERS_HEADERS}/Predicate.hpp
  ${PDAL_FILTERS_HEADERS}/Programmable.hpp
  
)

set (PDAL_PLANG_CPP 
  ${PDAL_PLANG_SRC}/AstNode.cpp
  ${PDAL_PLANG_SRC}/AstUtils.cpp
  ${PDAL_PLANG_SRC}/Parser.cpp
  ${PDAL_PLANG_SRC}/Parser_qi.cpp
  ${PDAL_PLANG_SRC}/Program.cpp
  ${PDAL_PLANG_SRC}/SymbolTable.cpp
  ${PDAL_FILTERS_SRC}/Predicate.cpp
  ${PDAL_FILTERS_SRC}/Programmable.cpp  
)

if(PDAL_HAVE_PLANG)
list (APPEND PDAL_CPP ${PDAL_PLANG_CPP} )
list (APPEND PDAL_HPP ${PDAL_PLANG_HPP} )
endif()

#
# config
#

set(PDAL_CONFIG_HPP
  ${PDAL_HEADERS_DIR}/export.hpp
  ${PDAL_HEADERS_DIR}/pdal.hpp
  ${PDAL_HEADERS_DIR}/pdal_internal.hpp
  ${PDAL_HEADERS_DIR}/pdal_config.hpp
  ${PDAL_HEADERS_DIR}/pdal_defines.h
)

set (PDAL_CONFIG_CPP 
  ${PROJECT_SOURCE_DIR}/src/pdal_config.cpp
)

list (APPEND PDAL_CPP ${PDAL_CONFIG_CPP} )
list (APPEND PDAL_HPP ${PDAL_CONFIG_HPP} )


#
# external/boost/uuid
#

set(PDAL_EXTERNAL_BOOST_UUID_HPP
  ${PDAL_HEADERS_DIR}/external/boost/uuid/uuid.hpp
  ${PDAL_HEADERS_DIR}/external/boost/uuid/name_generator.hpp
  ${PDAL_HEADERS_DIR}/external/boost/uuid/nil_generator.hpp
  ${PDAL_HEADERS_DIR}/external/boost/uuid/random_generator.hpp
  ${PDAL_HEADERS_DIR}/external/boost/uuid/seed_rng.hpp
  ${PDAL_HEADERS_DIR}/external/boost/uuid/sha1.hpp
  ${PDAL_HEADERS_DIR}/external/boost/uuid/string_generator.hpp
  ${PDAL_HEADERS_DIR}/external/boost/uuid/uuid.hpp
  ${PDAL_HEADERS_DIR}/external/boost/uuid/uuid_generators.hpp
  ${PDAL_HEADERS_DIR}/external/boost/uuid/uuid_io.hpp
  ${PDAL_HEADERS_DIR}/external/boost/uuid/uuid_serialize.hpp
)

list (APPEND PDAL_HPP ${PDAL_EXTERNAL_BOOST_UUID_HPP} )


#
# Group source files for IDE source explorers (e.g. Visual Studio)
#

source_group("CMake Files" FILES CMakeLists.txt)

source_group("Header Files" FILES ${PDAL_BASE_HPP})
source_group("Header Files\\config" FILES ${PDAL_CONFIG_HPP})
source_group("Header Files\\external\\boost\\uuid" FILES ${PDAL_EXTERNAL_BOOST_UUID_HPP})
source_group("Header Files\\drivers\\faux" FILES ${PDAL_DRIVERS_FAUX_HPP})
source_group("Header Files\\drivers\\las" FILES ${PDAL_DRIVERS_LAS_HPP})
source_group("Header Files\\drivers\\oci" FILES ${PDAL_DRIVERS_OCI_HPP})
source_group("Header Files\\drivers\\pipeline" FILES ${PDAL_DRIVERS_PIPELINE_HPP})
source_group("Header Files\\drivers\\qfit" FILES ${PDAL_DRIVERS_QFIT_HPP})
source_group("Header Files\\drivers\\terrasolid" FILES ${PDAL_DRIVERS_TERRASOLID_HPP})
source_group("Header Files\\filters" FILES ${PDAL_FILTERS_HPP})
source_group("Header Files\\plang" FILES ${PDAL_PLANG_HPP})

source_group("Source Files" FILES ${PDAL_BASE_CPP})
source_group("Source Files\\config" FILES ${PDAL_CONFIG_CPP})
source_group("Source Files\\drivers\\faux" FILES ${PDAL_DRIVERS_FAUX_CPP})
source_group("Source Files\\drivers\\las" FILES ${PDAL_DRIVERS_LAS_CPP})
source_group("Source Files\\drivers\\oci" FILES ${PDAL_DRIVERS_OCI_CPP})
source_group("Source Files\\drivers\\pipeline" FILES ${PDAL_DRIVERS_PIPELINE_CPP})
source_group("Source Files\\drivers\\qfit" FILES ${PDAL_DRIVERS_QFIT_CPP})
source_group("Source Files\\drivers\\terrasolid" FILES ${PDAL_DRIVERS_TERRASOLID_CPP})
source_group("Source Files\\filters" FILES ${PDAL_FILTERS_CPP})
source_group("Source Files\\plang" FILES ${PDAL_PLANG_CPP})


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
  ${TIFF_LIBRARY}
  ${ORACLE_LIBRARY}
  ${MRSID_LIBRARY}
  ${GEOTIFF_LIBRARY}
  ${GDAL_LIBRARY}
  ${LASZIP_LIBRARY}
  ${Boost_LIBRARIES}
  ${LIBXML2_LIBRARIES}
  ${P2G_LIBRARY})


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
