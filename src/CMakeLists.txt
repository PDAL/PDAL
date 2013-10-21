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

set(PDAL_HEADERS_DIR "${PROJECT_SOURCE_DIR}/include/pdal")
set(PDAL_HPP "")
set(PDAL_CPP "")

#
# base
#
set(PDAL_BASE_HPP
  "${PDAL_HEADERS_DIR}/pdal_error.hpp"
  "${PDAL_HEADERS_DIR}/pdal_types.hpp"
  "${PDAL_HEADERS_DIR}/Bounds.hpp"
  "${PDAL_HEADERS_DIR}/Dimension.hpp"
  "${PDAL_HEADERS_DIR}/FileUtils.hpp"
  "${PDAL_HEADERS_DIR}/Filter.hpp"
  "${PDAL_HEADERS_DIR}/FilterIterator.hpp"
  "${PDAL_HEADERS_DIR}/GDALUtils.hpp"
  "${PDAL_HEADERS_DIR}/GlobalEnvironment.hpp"
  "${PDAL_HEADERS_DIR}/gitsha.h"
  "${PDAL_HEADERS_DIR}/Log.hpp"
  "${PDAL_HEADERS_DIR}/Metadata.hpp"
  "${PDAL_HEADERS_DIR}/MultiFilter.hpp"
  "${PDAL_HEADERS_DIR}/MultiFilterIterator.hpp"
  "${PDAL_HEADERS_DIR}/Options.hpp"
  "${PDAL_HEADERS_DIR}/PipelineManager.hpp"
  "${PDAL_HEADERS_DIR}/PipelineReader.hpp"
  "${PDAL_HEADERS_DIR}/PipelineWriter.hpp"
  "${PDAL_HEADERS_DIR}/PointBuffer.hpp"  
  "${PDAL_HEADERS_DIR}/Range.hpp"
  "${PDAL_HEADERS_DIR}/Reader.hpp"
  "${PDAL_HEADERS_DIR}/ReaderIterator.hpp"
  "${PDAL_HEADERS_DIR}/Schema.hpp"
  "${PDAL_HEADERS_DIR}/SpatialReference.hpp"
  "${PDAL_HEADERS_DIR}/Stage.hpp"
  "${PDAL_HEADERS_DIR}/StageBase.hpp"
  "${PDAL_HEADERS_DIR}/StageFactory.hpp"
  "${PDAL_HEADERS_DIR}/StageInfo.hpp"
  "${PDAL_HEADERS_DIR}/StageIterator.hpp"
  "${PDAL_HEADERS_DIR}/StreamFactory.hpp"
  "${PDAL_HEADERS_DIR}/ThreadEnvironment.hpp"
  "${PDAL_HEADERS_DIR}/UserCallback.hpp"
  "${PDAL_HEADERS_DIR}/Utils.hpp"
  "${PDAL_HEADERS_DIR}/Vector.hpp"  
  "${PDAL_HEADERS_DIR}/Writer.hpp"
  "${PDAL_HEADERS_DIR}/XMLSchema.hpp"
  "${PDAL_HEADERS_DIR}/third/nanoflann.hpp"
  "${PDAL_HEADERS_DIR}/third/string_ref.hpp"
  "${PDAL_HEADERS_DIR}/third/string_ref_fwd.hpp"
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
  gitsha.cpp
  GlobalEnvironment.cpp
  Log.cpp
  Metadata.cpp
  MultiFilter.cpp
  MultiFilterIterator.cpp
#  Object.cpp
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
  StageInfo.cpp
  StageIterator.cpp
  StreamFactory.cpp
  ThreadEnvironment.cpp
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
# drivers/buffer
#
set(PDAL_BUFFER_PATH drivers/buffer)
set(PDAL_BUFFER_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_BUFFER_PATH}")
set(PDAL_BUFFER_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_BUFFER_PATH}")
set(PDAL_DRIVERS_BUFFER_HPP 
  "${PDAL_BUFFER_HEADERS}/Reader.hpp"
)

set (PDAL_DRIVERS_BUFFER_CPP
  "${PDAL_BUFFER_SRC}/Reader.cpp"
)
list (APPEND PDAL_CPP ${PDAL_DRIVERS_BUFFER_CPP} )
list (APPEND PDAL_HPP ${PDAL_DRIVERS_BUFFER_HPP} )


#
# drivers/faux
#
set(PDAL_FAUX_PATH drivers/faux)
set(PDAL_FAUX_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_FAUX_PATH}")
set(PDAL_FAUX_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_FAUX_PATH}")
set(PDAL_DRIVERS_FAUX_HPP 
  "${PDAL_FAUX_HEADERS}/Reader.hpp"
  "${PDAL_FAUX_HEADERS}/Writer.hpp"
)

set (PDAL_DRIVERS_FAUX_CPP
  "${PDAL_FAUX_SRC}/Reader.cpp"
  "${PDAL_FAUX_SRC}/Writer.cpp" 
)
list (APPEND PDAL_CPP ${PDAL_DRIVERS_FAUX_CPP} )
list (APPEND PDAL_HPP ${PDAL_DRIVERS_FAUX_HPP} )


#
# drivers/las
#
set(PDAL_LAS_PATH drivers/las)
set(PDAL_LAS_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_LAS_PATH}")
set(PDAL_LAS_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_LAS_PATH}")
set (PDAL_DRIVERS_LAS_HPP
  "${PDAL_LAS_SRC}/GeotiffSupport.hpp"
  "${PDAL_LAS_SRC}/LasHeaderReader.hpp"
  "${PDAL_LAS_SRC}/LasHeaderWriter.hpp"
  "${PDAL_LAS_SRC}/ZipPoint.hpp"
  "${PDAL_LAS_HEADERS}/Header.hpp"
  "${PDAL_LAS_HEADERS}/Reader.hpp"
  "${PDAL_LAS_HEADERS}/ReaderBase.hpp"
  "${PDAL_LAS_HEADERS}/SummaryData.hpp"
  "${PDAL_LAS_HEADERS}/Support.hpp"
  "${PDAL_LAS_HEADERS}/Writer.hpp"
  "${PDAL_LAS_HEADERS}/VariableLengthRecord.hpp"
)

if ((GEOTIFF_FOUND) AND (GDAL_FOUND))
    set (PDAL_DRIVERS_LAS_GTIFF "${PDAL_LAS_SRC}/GeotiffSupport.cpp")
endif()

if (LASZIP_FOUND)
    set(PDAL_DRIVERS_LAS_LASZIP "${PDAL_LAS_SRC}/ZipPoint.cpp")
endif()

set (PDAL_DRIVERS_LAS_CPP
  ${PDAL_DRIVERS_LAS_GTIFF}
  ${PDAL_DRIVERS_LAS_LASZIP}
  "${PDAL_LAS_SRC}/Header.cpp"
  "${PDAL_LAS_SRC}/LasHeaderReader.cpp"
  "${PDAL_LAS_SRC}/LasHeaderWriter.cpp"
  "${PDAL_LAS_SRC}/Reader.cpp"
  "${PDAL_LAS_SRC}/SummaryData.cpp"
  "${PDAL_LAS_SRC}/Support.cpp"
  "${PDAL_LAS_SRC}/VariableLengthRecord.cpp"
  "${PDAL_LAS_SRC}/Writer.cpp"
)
list (APPEND PDAL_CPP ${PDAL_DRIVERS_LAS_CPP} )
list (APPEND PDAL_HPP ${PDAL_DRIVERS_LAS_HPP} )

#
# drivers/mrsid
#
set(PDAL_MRSID_PATH drivers/mrsid)
set(PDAL_MRSID_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_MRSID_PATH}")
set(PDAL_MRSID_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_MRSID_PATH}")

set(PDAL_DRIVERS_MRSID_HPP
  "${PDAL_MRSID_HEADERS}/Reader.hpp"
)

set (PDAL_DRIVERS_MRSID_CPP 
  "${PDAL_MRSID_SRC}/Reader.cpp"
)

if (MRSID_FOUND)
    list (APPEND PDAL_CPP ${PDAL_DRIVERS_MRSID_CPP} )
    list (APPEND PDAL_HPP ${PDAL_DRIVERS_MRSID_HPP} )
endif()


#
# drivers/caris
#
set(PDAL_CARIS_PATH drivers/caris)
set(PDAL_CARIS_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_CARIS_PATH}")
set(PDAL_CARIS_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_CARIS_PATH}")

set(PDAL_DRIVERS_CARIS_HPP
  "${PDAL_CARIS_HEADERS}/CloudReader.hpp"
  "${PDAL_CARIS_HEADERS}/CloudIterator.hpp"
  "${PDAL_CARIS_SRC}/Utils.hpp"
  "${PDAL_CARIS_SRC}/config.h"
  "${PDAL_CARIS_SRC}/caris/caris_pc_wrapper.h"
)

set (PDAL_DRIVERS_CARIS_CPP 
  "${PDAL_CARIS_SRC}/CloudReader.cpp"
  "${PDAL_CARIS_SRC}/CloudIterator.cpp"
  "${PDAL_CARIS_SRC}/pdal_csar.cpp"
  "${PDAL_CARIS_SRC}/Utils.cpp"
  "${PDAL_CARIS_SRC}/caris/caris_pc_wrapper.c"
)

if (CARIS_FOUND)
    list (APPEND PDAL_CPP ${PDAL_DRIVERS_CARIS_CPP} )
    list (APPEND PDAL_HPP ${PDAL_DRIVERS_CARIS_HPP} )
endif()


#
# drivers/oci
#

set(PDAL_ORACLE_PATH drivers/oci)
set(PDAL_ORACLE_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_ORACLE_PATH}")
set(PDAL_ORACLE_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_ORACLE_PATH}")

set(PDAL_DRIVERS_OCI_HPP
  "${PDAL_ORACLE_HEADERS}/Writer.hpp"
  "${PDAL_ORACLE_HEADERS}/Reader.hpp"
  "${PDAL_ORACLE_HEADERS}/oci_wrapper.h"
  "${PDAL_ORACLE_HEADERS}/common.hpp"
)

set (PDAL_DRIVERS_OCI_CPP 
  "${PDAL_ORACLE_SRC}/oci_wrapper.cpp"
  "${PDAL_ORACLE_SRC}/common.cpp"
  "${PDAL_ORACLE_SRC}/Reader.cpp"
  "${PDAL_ORACLE_SRC}/Writer.cpp"
)
   
if (ORACLE_FOUND)
if (NOT USE_PDAL_PLUGIN_OCI)
    list (APPEND PDAL_CPP ${PDAL_DRIVERS_OCI_CPP} )
    list (APPEND PDAL_HPP ${PDAL_DRIVERS_OCI_HPP} )
endif()
endif()


#
# drivers/pipeline
#

set(PDAL_PIPELINE_PATH drivers/pipeline)
set(PDAL_PIPELINE_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_PIPELINE_PATH}")
set(PDAL_PIPELINE_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_PIPELINE_PATH}")

set (PDAL_DRIVERS_PIPELINE_HPP
  "${PDAL_PIPELINE_HEADERS}/Reader.hpp"
)

set (PDAL_DRIVERS_PIPELINE_CPP 
  "${PDAL_PIPELINE_SRC}/Reader.cpp"
)

list (APPEND PDAL_CPP ${PDAL_DRIVERS_PIPELINE_CPP} )
list (APPEND PDAL_HPP ${PDAL_DRIVERS_PIPELINE_HPP} )


#
# drivers/qfit
#
set(PDAL_QFIT_PATH drivers/qfit)
set(PDAL_QFIT_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_QFIT_PATH}")
set(PDAL_QFIT_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_QFIT_PATH}")

set(PDAL_DRIVERS_QFIT_HPP
  "${PDAL_QFIT_HEADERS}/Reader.hpp"
)

set (PDAL_DRIVERS_QFIT_CPP 
  "${PDAL_QFIT_SRC}/Reader.cpp"
)

list (APPEND PDAL_CPP ${PDAL_DRIVERS_QFIT_CPP} )
list (APPEND PDAL_HPP ${PDAL_DRIVERS_QFIT_HPP} )

#
# drivers/terrasolid
#
set(PDAL_TERRASOLID_PATH drivers/terrasolid)
set(PDAL_TERRASOLID_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_TERRASOLID_PATH}")
set(PDAL_TERRASOLID_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_TERRASOLID_PATH}")

set(PDAL_DRIVERS_TERRASOLID_HPP
  "${PDAL_TERRASOLID_HEADERS}/Reader.hpp"
)

set (PDAL_DRIVERS_TERRASOLID_CPP 
  "${PDAL_TERRASOLID_SRC}/Reader.cpp"
)

list (APPEND PDAL_CPP ${PDAL_DRIVERS_TERRASOLID_CPP} )
list (APPEND PDAL_HPP ${PDAL_DRIVERS_TERRASOLID_HPP} )

#
# drivers/p2g
#

set(PDAL_P2G_PATH drivers/p2g)
set(PDAL_P2G_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_P2G_PATH}")
set(PDAL_P2G_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_P2G_PATH}")

set(PDAL_DRIVERS_P2G_HPP
  "${PDAL_P2G_HEADERS}/Writer.hpp"
)

set (PDAL_DRIVERS_P2G_CPP 
  "${PDAL_P2G_SRC}/Writer.cpp"
)
   
if (P2G_FOUND)
    list (APPEND PDAL_CPP ${PDAL_DRIVERS_P2G_CPP} )
    list (APPEND PDAL_HPP ${PDAL_DRIVERS_P2G_HPP} )
endif()

#
# drivers/text
#
set(PDAL_TEXT_PATH drivers/text)
set(PDAL_TEXT_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_TEXT_PATH}")
set(PDAL_TEXT_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_TEXT_PATH}")

set(PDAL_DRIVERS_TEXT_HPP
  "${PDAL_TEXT_HEADERS}/Writer.hpp"
)

set (PDAL_DRIVERS_TEXT_CPP 
  ${PDAL_TEXT_SRC}/Writer.cpp
)

if (NOT USE_PDAL_PLUGIN_TEXT)
    list (APPEND PDAL_CPP ${PDAL_DRIVERS_TEXT_CPP} )
    list (APPEND PDAL_HPP ${PDAL_DRIVERS_TEXT_HPP} )
endif()

#
# drivers/nitf
#
set(PDAL_NITF_PATH drivers/nitf)
set(PDAL_NITF_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_NITF_PATH}")
set(PDAL_NITF_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_NITF_PATH}")

set(PDAL_DRIVERS_NITF_READER_HPP
  "${PDAL_NITF_SRC}/nitflib.h"
  "${PDAL_NITF_SRC}/NitfFile.hpp"
  "${PDAL_NITF_HEADERS}/Reader.hpp"
)

set (PDAL_DRIVERS_NITF_READER_CPP 
  "${PDAL_NITF_SRC}/NitfFile.cpp"
  "${PDAL_NITF_SRC}/Reader.cpp"
)

if (GDAL_FOUND)
    list (APPEND PDAL_CPP ${PDAL_DRIVERS_NITF_READER_CPP} )
    list (APPEND PDAL_HPP ${PDAL_DRIVERS_NITF_READER_HPP} )
endif()

set(PDAL_DRIVERS_NITF_WRITER_HPP
  "${PDAL_NITF_HEADERS}/Writer.hpp"
)

set (PDAL_DRIVERS_NITF_WRITER_CPP 
  "${PDAL_NITF_SRC}/Writer.cpp"
)

if (NITRO_FOUND)
    list (APPEND PDAL_CPP ${PDAL_DRIVERS_NITF_WRITER_CPP} )
    list (APPEND PDAL_HPP ${PDAL_DRIVERS_NITF_WRITER_HPP} )
endif()


#
# drivers/soci
#
set(PDAL_SOCI_PATH drivers/soci)
set(PDAL_SOCI_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_SOCI_PATH}")
set(PDAL_SOCI_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_SOCI_PATH}")

set(PDAL_DRIVERS_SOCI_HPP
  "${PDAL_SOCI_HEADERS}/common.hpp"
  "${PDAL_SOCI_HEADERS}/Reader.hpp"
  "${PDAL_SOCI_HEADERS}/Writer.hpp"
)

set (PDAL_DRIVERS_SOCI_CPP 
  "${PDAL_SOCI_SRC}/Reader.cpp"
  "${PDAL_SOCI_SRC}/Writer.cpp"
)

if (SOCI_FOUND)
    if (NOT USE_PDAL_PLUGIN_SOCI)
        list (APPEND PDAL_CPP ${PDAL_DRIVERS_SOCI_CPP} )
        list (APPEND PDAL_HPP ${PDAL_DRIVERS_SOCI_HPP} )
    endif()
endif()


#
# drivers/pgpointcloud
# we require PostgreSQL
#
set(PDAL_PGPOINTCLOUD_PATH drivers/pgpointcloud)
set(PDAL_PGPOINTCLOUD_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_PGPOINTCLOUD_PATH}")
set(PDAL_PGPOINTCLOUD_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_PGPOINTCLOUD_PATH}")

set(PDAL_DRIVERS_PGPOINTCLOUD_HPP
  "${PDAL_PGPOINTCLOUD_HEADERS}/common.hpp"
  "${PDAL_PGPOINTCLOUD_HEADERS}/Writer.hpp"
  "${PDAL_PGPOINTCLOUD_HEADERS}/Reader.hpp"
)

set (PDAL_DRIVERS_PGPOINTCLOUD_CPP 
  "${PDAL_PGPOINTCLOUD_SRC}/Writer.cpp"
  "${PDAL_PGPOINTCLOUD_SRC}/Reader.cpp"
)

# no provision is made for building as a plugin
if (POSTGRESQL_FOUND)
  if ( USE_PDAL_PLUGIN_PGPOINTCLOUD )
    message(FATAL_ERROR, "PgPointCloud plugin support unimplemented")
  else()
    list (APPEND PDAL_CPP ${PDAL_DRIVERS_PGPOINTCLOUD_CPP} )
    list (APPEND PDAL_HPP ${PDAL_DRIVERS_PGPOINTCLOUD_HPP} )
  endif()
endif()


#
# filters
#

set(PDAL_FILTERS_PATH filters)
set(PDAL_FILTERS_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_FILTERS_PATH}")
set(PDAL_FILTERS_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_FILTERS_PATH}")

set(PDAL_FILTERS_HPP
  "${PDAL_FILTERS_HEADERS}/ByteSwap.hpp"
  "${PDAL_FILTERS_HEADERS}/Cache.hpp"
  "${PDAL_FILTERS_HEADERS}/Chipper.hpp"
  "${PDAL_FILTERS_HEADERS}/Color.hpp"
  "${PDAL_FILTERS_HEADERS}/Colorization.hpp"
  "${PDAL_FILTERS_HEADERS}/Crop.hpp"
  "${PDAL_FILTERS_HEADERS}/Decimation.hpp"
  "${PDAL_FILTERS_HEADERS}/HexBin.hpp"
  "${PDAL_FILTERS_HEADERS}/InPlaceReprojection.hpp"
  "${PDAL_FILTERS_HEADERS}/Mosaic.hpp"
  "${PDAL_FILTERS_HEADERS}/PointBufferCache.hpp"
  "${PDAL_FILTERS_HEADERS}/Predicate.hpp"
  "${PDAL_FILTERS_HEADERS}/Programmable.hpp"
  "${PDAL_FILTERS_HEADERS}/Reprojection.hpp"
  "${PDAL_FILTERS_HEADERS}/Scaling.hpp"
  "${PDAL_FILTERS_HEADERS}/Selector.hpp"
  "${PDAL_FILTERS_HEADERS}/Stats.hpp"
)

set (PDAL_FILTERS_CPP 
  "${PDAL_FILTERS_SRC}/ByteSwap.cpp"
  "${PDAL_FILTERS_SRC}/Cache.cpp"
  "${PDAL_FILTERS_SRC}/Chipper.cpp"
  "${PDAL_FILTERS_SRC}/Color.cpp"
  "${PDAL_FILTERS_SRC}/Colorization.cpp"
  "${PDAL_FILTERS_SRC}/Crop.cpp"
  "${PDAL_FILTERS_SRC}/Decimation.cpp"
  "${PDAL_FILTERS_SRC}/HexBin.cpp"
  "${PDAL_FILTERS_SRC}/InPlaceReprojection.cpp"
  "${PDAL_FILTERS_SRC}/Mosaic.cpp"
  "${PDAL_FILTERS_SRC}/PointBufferCache.cpp"
  "${PDAL_FILTERS_SRC}/Predicate.cpp"
  "${PDAL_FILTERS_SRC}/Programmable.cpp"  
  "${PDAL_FILTERS_SRC}/Reprojection.cpp"
  "${PDAL_FILTERS_SRC}/Scaling.cpp"
  "${PDAL_FILTERS_SRC}/Selector.cpp"
  "${PDAL_FILTERS_SRC}/Stats.cpp"
)

list (APPEND PDAL_CPP ${PDAL_FILTERS_CPP} )
list (APPEND PDAL_HPP ${PDAL_FILTERS_HPP} )


#
# plag parser
#

set(PDAL_PLANG_PATH plang)
set(PDAL_PLANG_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_PLANG_PATH}")
set(PDAL_PLANG_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_PLANG_PATH}")

set(PDAL_PLANG_HPP
  "${PDAL_PLANG_HEADERS}/BufferedInvocation.hpp"
  "${PDAL_PLANG_HEADERS}/PythonEnvironment.hpp"
  "${PDAL_PLANG_HEADERS}/Invocation.hpp"
  "${PDAL_PLANG_SRC}/Redirector.hpp"
  "${PDAL_PLANG_HEADERS}/Script.hpp"
)

set (PDAL_PLANG_CPP 
  "${PDAL_PLANG_SRC}/BufferedInvocation.cpp"
  "${PDAL_PLANG_SRC}/PythonEnvironment.cpp"
  "${PDAL_PLANG_SRC}/Invocation.cpp"
  "${PDAL_PLANG_SRC}/Redirector.cpp"
  "${PDAL_PLANG_SRC}/Script.cpp"
)

list (APPEND PDAL_CPP ${PDAL_PLANG_CPP} )
list (APPEND PDAL_HPP ${PDAL_PLANG_HPP} )

#
# config
#

set(PDAL_CONFIG_HPP
  "${PDAL_HEADERS_DIR}/pdal_export.hpp"
  "${PDAL_HEADERS_DIR}/pdal_internal.hpp"
  "${PDAL_HEADERS_DIR}/pdal_config.hpp"
  "${PROJECT_BINARY_DIR}/include/pdal/pdal_defines.h"
)

set (PDAL_CONFIG_CPP 
  "${PROJECT_SOURCE_DIR}/src/pdal_config.cpp"
)

list (APPEND PDAL_CPP ${PDAL_CONFIG_CPP} )
list (APPEND PDAL_HPP ${PDAL_CONFIG_HPP} )


set(PDAL_KERNEL_PATH kernel)
set(PDAL_KERNEL_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_KERNEL_PATH}")
set(PDAL_KERNEL_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_KERNEL_PATH}")

set(PDAL_KERNEL_HPP
    ${PDAL_KERNEL_HEADERS}/Application.hpp 
    ${PDAL_KERNEL_HEADERS}/Support.hpp
    ${PDAL_KERNEL_HEADERS}/Info.hpp
    ${PDAL_KERNEL_HEADERS}/Kernel.hpp
    ${PDAL_KERNEL_HEADERS}/Pipeline.hpp
    ${PDAL_KERNEL_HEADERS}/Query.hpp
    ${PDAL_KERNEL_HEADERS}/Translate.hpp
)

set(PDAL_KERNEL_CPP
    ${PDAL_KERNEL_SRC}/Application.cpp
    ${PDAL_KERNEL_SRC}/Support.cpp
    ${PDAL_KERNEL_SRC}/Info.cpp
    ${PDAL_KERNEL_SRC}/Pipeline.cpp
    ${PDAL_KERNEL_SRC}/Query.cpp
    ${PDAL_KERNEL_SRC}/Translate.cpp
)

list (APPEND PDAL_CPP ${PDAL_KERNEL_CPP} )
list (APPEND PDAL_HPP ${PDAL_KERNEL_HPP} )
#
# Group source files for IDE source explorers (e.g. Visual Studio)
#

source_group("CMake Files" FILES CMakeLists.txt)

source_group("Header Files" FILES ${PDAL_BASE_HPP})
source_group("Header Files\\config" FILES ${PDAL_CONFIG_HPP})
source_group("Header Files\\drivers\\buffer" FILES ${PDAL_DRIVERS_BUFFER_HPP})
source_group("Header Files\\drivers\\faux" FILES ${PDAL_DRIVERS_FAUX_HPP})
source_group("Header Files\\drivers\\caris" FILES ${PDAL_DRIVERS_CARIS_HPP})
source_group("Header Files\\drivers\\las" FILES ${PDAL_DRIVERS_LAS_HPP})
source_group("Header Files\\drivers\\nitf" FILES ${PDAL_DRIVERS_NITF_HPP})
source_group("Header Files\\drivers\\oci" FILES ${PDAL_DRIVERS_OCI_HPP})
source_group("Header Files\\drivers\\pipeline" FILES ${PDAL_DRIVERS_PIPELINE_HPP})
source_group("Header Files\\drivers\\qfit" FILES ${PDAL_DRIVERS_QFIT_HPP})
source_group("Header Files\\drivers\\terrasolid" FILES ${PDAL_DRIVERS_TERRASOLID_HPP})
source_group("Header Files\\drivers\\text" FILES ${PDAL_DRIVERS_TEXT_HPP})
source_group("Header Files\\drivers\\pgpointcloud" FILES ${PDAL_DRIVERS_PGPOINTCLOUD_HPP})
source_group("Header Files\\filters" FILES ${PDAL_FILTERS_HPP})
source_group("Header Files\\plang" FILES ${PDAL_PLANG_HPP})

source_group("Source Files" FILES ${PDAL_BASE_CPP})
source_group("Source Files\\config" FILES ${PDAL_CONFIG_CPP})
source_group("Source Files\\drivers\\buffer" FILES ${PDAL_DRIVERS_BUFFER_CPP})
source_group("Source Files\\drivers\\faux" FILES ${PDAL_DRIVERS_FAUX_CPP})
source_group("Source Files\\drivers\\caris" FILES ${PDAL_DRIVERS_CARIS_CPP})
source_group("Source Files\\drivers\\las" FILES ${PDAL_DRIVERS_LAS_CPP})
source_group("Source Files\\drivers\\nitf" FILES ${PDAL_DRIVERS_NITF_CPP})
source_group("Source Files\\drivers\\oci" FILES ${PDAL_DRIVERS_OCI_CPP})
source_group("Source Files\\drivers\\pipeline" FILES ${PDAL_DRIVERS_PIPELINE_CPP})
source_group("Source Files\\drivers\\qfit" FILES ${PDAL_DRIVERS_QFIT_CPP})
source_group("Source Files\\drivers\\terrasolid" FILES ${PDAL_DRIVERS_TERRASOLID_CPP})
source_group("Source Files\\drivers\\text" FILES ${PDAL_DRIVERS_TEXT_CPP})
source_group("Source Files\\drivers\\pgpointcloud" FILES ${PDAL_DRIVERS_PGPOINTCLOUD_CPP})
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

# see https://github.com/PDAL/PDAL/issues/108 for discussion on this
#SET_DIRECTORY_PROPERTIES(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "../pdal_defines.h;../include/pdal/pdal_defines.h")

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

if (PDAL_BUILD_STATIC)
  set(PDAL_LIB_TYPE STATIC)
else()
  set(PDAL_LIB_TYPE SHARED)
endif()
add_library(${PDAL_LIB_NAME} ${PDAL_LIB_TYPE} ${PDAL_SOURCES})

set (SOCI_LIBRARIES ${SOCI_LIBRARY} ${SOCI_postgresql_PLUGIN} ${SOCI_sqlite3_PLUGIN} ${SQLITE3_LIBRARY})

if (WITH_SOCI)
if (NOT USE_PDAL_PLUGIN_SOCI)
target_link_libraries(${APPS_CPP_DEPENDENCIES} "${SOCI_LIBRARIES}")
endif()
endif()

if (WITH_PGPOINTCLOUD)
if (NOT USE_PDAL_PLUGIN_SOCI)
target_link_libraries(${APPS_CPP_DEPENDENCIES} "${POSTGRESQL_LIBRARIES}")
endif()
endif()

if (WITH_ORACLE)
if (NOT USE_PDAL_PLUGIN_OCI)
target_link_libraries(${APPS_CPP_DEPENDENCIES} "${ORACLE_LIBRARY}")
endif()
endif()

if (WITH_NITRO)
if (NOT USE_PDAL_PLUGIN_NITRO)
target_link_libraries(${APPS_CPP_DEPENDENCIES} "${NITRO_LIBRARIES}")
endif()
endif()

if (WIN32)
  target_link_libraries(${APPS_CPP_DEPENDENCIES} LINK_PUBLIC general
    "${PDAL_LIB_NAME}"
    "${TIFF_LIBRARY}"
    "${MRSID_LIBRARY}"
    "${GEOTIFF_LIBRARY}"
    "${GDAL_LIBRARY}"
    "${LASZIP_LIBRARY}"
    "${LIBXML2_LIBRARIES}"
    "${PYTHON_LIBRARY}"
    "${P2G_LIBRARY}"
    "${GEOS_LIBRARY}"
    "${HEXER_LIBRARY}"
    "${ORACLE_LIBRARY}")
else()
  target_link_libraries(${APPS_CPP_DEPENDENCIES}
    "${PDAL_LIB_NAME}"
    "${CMAKE_THREAD_LIBS_INIT}"
    "${CMAKE_DL_LIBS}"
    "${TIFF_LIBRARY}"
    "${MRSID_LIBRARY}"
    "${GEOTIFF_LIBRARY}"
    "${GDAL_LIBRARY}"
    "${LASZIP_LIBRARY}"
    ${LIBXML2_LIBRARIES}
    "${PYTHON_LIBRARY}"
    "${P2G_LIBRARY}"
    "${GEOS_LIBRARY}"
    "${HEXER_LIBRARY}"
    "${ORACLE_LIBRARY}"
    ${NITRO_LIBRARIES}
    ${SOCI_LIBRARIES})
endif()

message(STATUS "Using boost lib: ${Boost_LIBRARIES}")

if (WIN32)
	if (CMAKE_VERSION VERSION_GREATER 2.8.6)
		target_link_libraries(${APPS_CPP_DEPENDENCIES}
                       LINK_PRIVATE general ${Boost_LIBRARIES} "${Boost_FILESYSTEM_LIBRARY}")
	else()
		target_link_libraries(${APPS_CPP_DEPENDENCIES}
                        ${Boost_LIBRARIES} "${Boost_FILESYSTEM_LIBRARY}")
	endif()
else()
	target_link_libraries(${APPS_CPP_DEPENDENCIES}
                      ${Boost_LIBRARIES} )

endif(WIN32)

set_target_properties(${PDAL_LIB_NAME}
  PROPERTIES SOVERSION "${PDAL_LIB_SOVERSION}" )

if (APPLE)
  set_target_properties(
    ${PDAL_LIB_NAME}
    PROPERTIES
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()


if (USE_PDAL_PLUGIN_TEXT)
    set(PDAL_TEXT_WRITER_LIB_NAME pdal_plugin_writer_text)
    add_library(${PDAL_TEXT_WRITER_LIB_NAME} SHARED "${PDAL_DRIVERS_TEXT_CPP}" "${PDAL_DRIVERS_TEXT_HPP}")
    target_link_libraries(${PDAL_TEXT_WRITER_LIB_NAME}
                          "${PDAL_LIB_NAME}")
    set_target_properties("${PDAL_TEXT_WRITER_LIB_NAME}"
    PROPERTIES SOVERSION "${PDAL_LIB_SOVERSION}" )
    if (APPLE)
    set_target_properties(
      "${PDAL_TEXT_WRITER_LIB_NAME}"
      PROPERTIES
      INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
    endif()
endif()

if (WITH_SOCI)
if (USE_PDAL_PLUGIN_SOCI)
    set(PDAL_SOCI_WRITER_LIB_NAME pdal_plugin_writer_soci)
    add_library(${PDAL_SOCI_WRITER_LIB_NAME} SHARED "${PDAL_SOCI_SRC}/Writer.cpp")
    target_link_libraries(${PDAL_SOCI_WRITER_LIB_NAME}
                          "${PDAL_LIB_NAME}"
                          "${SOCI_LIBRARIES}")
    set_target_properties(${PDAL_SOCI_WRITER_LIB_NAME}
    PROPERTIES SOVERSION "${PDAL_LIB_SOVERSION}" )
    if (APPLE)
    set_target_properties(
      "${PDAL_SOCI_WRITER_LIB_NAME}"
      PROPERTIES
      INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
    endif()

    set(PDAL_SOCI_READER_LIB_NAME pdal_plugin_reader_soci)
    add_library(${PDAL_SOCI_READER_LIB_NAME} SHARED ${PDAL_SOCI_SRC}/Reader.cpp)
    target_link_libraries(${PDAL_SOCI_READER_LIB_NAME}
                          "${PDAL_LIB_NAME}"
                          "${SOCI_LIBRARIES}")
    set_target_properties(${PDAL_SOCI_READER_LIB_NAME}
    PROPERTIES SOVERSION "${PDAL_LIB_SOVERSION}" )
    if (APPLE)
    set_target_properties(
      "${PDAL_SOCI_READER_LIB_NAME}"
      PROPERTIES
      INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
    endif()
    
endif()
endif()

if (WITH_ORACLE)
if (USE_PDAL_PLUGIN_OCI)
    set(PDAL_OCI_WRITER_LIB_NAME pdal_plugin_writer_oci)
    add_library(${PDAL_OCI_WRITER_LIB_NAME} SHARED "${PDAL_ORACLE_SRC}/oci_wrapper.cpp"
                                                    "${PDAL_ORACLE_SRC}/common.cpp"
                                                    "${PDAL_ORACLE_SRC}/Writer.cpp")
    target_link_libraries(${PDAL_OCI_WRITER_LIB_NAME}
                          "${PDAL_LIB_NAME}"
                          "${ORACLE_LIBRARY}")
    set_target_properties(${PDAL_OCI_WRITER_LIB_NAME}
    PROPERTIES SOVERSION "${PDAL_LIB_SOVERSION}" )
    if (APPLE)
    set_target_properties(
      "${PDAL_OCI_WRITER_LIB_NAME}"
      PROPERTIES
      INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
    endif()
endif()
endif()
###############################################################################
# Targets installation

install(TARGETS ${PDAL_LIB_NAME} ${PDAL_C_LIB_NAME}
  RUNTIME DESTINATION ${PDAL_BIN_DIR}
  LIBRARY DESTINATION ${PDAL_LIB_DIR}
  ARCHIVE DESTINATION ${PDAL_LIB_DIR})

install(DIRECTORY "${PDAL_HEADERS_DIR}/"
  DESTINATION "${PDAL_INCLUDE_DIR}"
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
