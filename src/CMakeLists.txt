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
  "${PDAL_HEADERS_DIR}/Charbuf.hpp"
  "${PDAL_HEADERS_DIR}/Dimension.hpp"
  "${PDAL_HEADERS_DIR}/Drivers.hpp"
  "${PDAL_HEADERS_DIR}/FileUtils.hpp"
  "${PDAL_HEADERS_DIR}/Filter.hpp"
  "${PDAL_HEADERS_DIR}/Filters.hpp"
  "${PDAL_HEADERS_DIR}/GlobalEnvironment.hpp"
  "${PDAL_HEADERS_DIR}/gitsha.h"
  "${PDAL_HEADERS_DIR}/IStream.hpp"
  "${PDAL_HEADERS_DIR}/KDIndex.hpp"
  "${PDAL_HEADERS_DIR}/Kernel.hpp"
  "${PDAL_HEADERS_DIR}/Log.hpp"
  "${PDAL_HEADERS_DIR}/Metadata.hpp"
  "${PDAL_HEADERS_DIR}/Options.hpp"
  "${PDAL_HEADERS_DIR}/OStream.hpp"
  "${PDAL_HEADERS_DIR}/PipelineManager.hpp"
  "${PDAL_HEADERS_DIR}/PipelineReader.hpp"
  "${PDAL_HEADERS_DIR}/PipelineWriter.hpp"
  "${PDAL_HEADERS_DIR}/PointBuffer.hpp"
  "${PDAL_HEADERS_DIR}/PointContext.hpp"
  "${PDAL_HEADERS_DIR}/QuadIndex.hpp"
  "${PDAL_HEADERS_DIR}/Range.hpp"
  "${PDAL_HEADERS_DIR}/RawPtBuf.hpp"
  "${PDAL_HEADERS_DIR}/Reader.hpp"
  "${PDAL_HEADERS_DIR}/ReaderIterator.hpp"
  "${PDAL_HEADERS_DIR}/SpatialReference.hpp"
  "${PDAL_HEADERS_DIR}/Stage.hpp"
  "${PDAL_HEADERS_DIR}/StageFactory.hpp"
  "${PDAL_HEADERS_DIR}/StageInfo.hpp"
  "${PDAL_HEADERS_DIR}/StageIterator.hpp"
  "${PDAL_HEADERS_DIR}/StageRunner.hpp"
  "${PDAL_HEADERS_DIR}/StreamFactory.hpp"
  "${PDAL_HEADERS_DIR}/UserCallback.hpp"
  "${PDAL_HEADERS_DIR}/Utils.hpp"
  "${PDAL_HEADERS_DIR}/Vector.hpp"
  "${PDAL_HEADERS_DIR}/Writer.hpp"
  "${PDAL_HEADERS_DIR}/XMLSchema.hpp"
  "${PDAL_HEADERS_DIR}/third/nanoflann.hpp"
  "${PDAL_HEADERS_DIR}/third/md5-pdal.h"
  "${PDAL_HEADERS_DIR}/third/string_ref.hpp"
  "${PDAL_HEADERS_DIR}/third/string_ref_fwd.hpp"
)

if (PDAL_HAVE_LIBXML2)
  SET (PDAL_XML_SRC XMLSchema.cpp)
endif()

set(PDAL_BASE_CPP
  Bounds.cpp
  Charbuf.cpp
  FileUtils.cpp
  Filter.cpp
  gitsha.cpp
  GlobalEnvironment.cpp
  KDIndex.cpp
  Log.cpp
  Metadata.cpp
  Options.cpp
  PipelineManager.cpp
  PipelineReader.cpp
  PipelineWriter.cpp
  PointBuffer.cpp
  QuadIndex.cpp
  Range.cpp
  Reader.cpp
  ReaderIterator.cpp
  SpatialReference.cpp
  Stage.cpp
  StageFactory.cpp
  StageInfo.cpp
  StageIterator.cpp
  StreamFactory.cpp
  Utils.cpp
  Vector.cpp
  Writer.cpp
  third/md5-pdal.c
  ${PDAL_XML_SRC}
)

list (APPEND PDAL_CPP ${PDAL_BASE_CPP} )
list (APPEND PDAL_HPP ${PDAL_BASE_HPP} )

#
# drivers/faux
#
set(PDAL_FAUX_PATH drivers/faux)
set(PDAL_FAUX_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_FAUX_PATH}")
set(PDAL_FAUX_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_FAUX_PATH}")
set(PDAL_DRIVERS_FAUX_HPP
  "${PDAL_FAUX_HEADERS}/Reader.hpp"
)

set (PDAL_DRIVERS_FAUX_CPP
  "${PDAL_FAUX_SRC}/Reader.cpp"
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
# drivers/bpf
#
set(PDAL_BPF_PATH drivers/bpf)
set(PDAL_BPF_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_BPF_PATH}")
set(PDAL_BPF_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_BPF_PATH}")

set(PDAL_DRIVERS_BPF_HPP
    "${PDAL_BPF_HEADERS}/BpfReader.hpp"
    "${PDAL_BPF_HEADERS}/BpfSeqIterator.hpp"
    "${PDAL_BPF_HEADERS}/BpfHeader.hpp"
)

set(PDAL_DRIVERS_BPF_CPP
    "${PDAL_BPF_SRC}/BpfHeader.cpp"
    "${PDAL_BPF_SRC}/BpfReader.cpp"
    "${PDAL_BPF_SRC}/BpfSeqIterator.cpp"
)

list (APPEND PDAL_CPP ${PDAL_DRIVERS_BPF_CPP} )
list (APPEND PDAL_HPP ${PDAL_DRIVERS_BPF_HPP} )

#
# WebSocketClient
#
set(PDAL_WEBSOCKET_CLIENT_HPP
    "${PDAL_HEADERS_DIR}/WebSocketClient.hpp"
)

set(PDAL_WEBSOCKET_CLIENT_CPP
    "${PDAL_SOURCE_DIR}/src/WebSocketClient.cpp"
)

if (WEBSOCKETPP_FOUND AND JSONCPP_FOUND)
    list (APPEND PDAL_CPP ${PDAL_WEBSOCKET_CLIENT_CPP})
    list (APPEND PDAL_HPP ${PDAL_WEBSOCKET_CLIENT_HPP})
endif()

#
# drivers/greyhound - requires WebSocketClient
#
set(PDAL_GREYHOUND_PATH drivers/greyhound)
set(PDAL_GREYHOUND_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_GREYHOUND_PATH}")
set(PDAL_GREYHOUND_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_GREYHOUND_PATH}")

set(PDAL_DRIVERS_GREYHOUND_HPP
    "${PDAL_GREYHOUND_HEADERS}/Reader.hpp"
)

set(PDAL_DRIVERS_GREYHOUND_CPP
    "${PDAL_GREYHOUND_SRC}/Reader.cpp"
)

if (WEBSOCKETPP_FOUND AND JSONCPP_FOUND)
    list (APPEND PDAL_CPP ${PDAL_DRIVERS_GREYHOUND_CPP} )
    list (APPEND PDAL_HPP ${PDAL_DRIVERS_GREYHOUND_HPP} )
endif()

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
  "${PDAL_ORACLE_HEADERS}/OciReader.hpp"
  "${PDAL_ORACLE_HEADERS}/OciSeqIterator.hpp"
  "${PDAL_ORACLE_HEADERS}/oci_wrapper.h"
  "${PDAL_ORACLE_HEADERS}/common.hpp"
)

set (PDAL_DRIVERS_OCI_CPP
  "${PDAL_ORACLE_SRC}/oci_wrapper.cpp"
  "${PDAL_ORACLE_SRC}/common.cpp"
  "${PDAL_ORACLE_SRC}/OciReader.cpp"
  "${PDAL_ORACLE_SRC}/OciSeqIterator.cpp"
  "${PDAL_ORACLE_SRC}/Writer.cpp"
)

if (ORACLE_FOUND)
if (NOT USE_PDAL_PLUGIN_OCI)
    list (APPEND PDAL_CPP ${PDAL_DRIVERS_OCI_CPP} )
    list (APPEND PDAL_HPP ${PDAL_DRIVERS_OCI_HPP} )
endif()
endif()


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
  "${PDAL_P2G_HEADERS}/P2gWriter.hpp"
)

set (PDAL_DRIVERS_P2G_CPP 
  "${PDAL_P2G_SRC}/P2gWriter.cpp"
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
# drivers/sqlite
#
set(PDAL_SQLITE_PATH drivers/sqlite)
set(PDAL_SQLITE_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_SQLITE_PATH}")
set(PDAL_SQLITE_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_SQLITE_PATH}")

set(PDAL_DRIVERS_SQLITE_HPP
  "${PDAL_SQLITE_HEADERS}/SQLiteCommon.hpp"
  "${PDAL_SQLITE_HEADERS}/SQLiteIterator.hpp"
  "${PDAL_SQLITE_HEADERS}/SQLiteReader.hpp"
  "${PDAL_SQLITE_HEADERS}/SQLiteWriter.hpp"
)

set (PDAL_DRIVERS_SQLITE_CPP
  "${PDAL_SQLITE_SRC}/SQLiteIterator.cpp"
  "${PDAL_SQLITE_SRC}/SQLiteReader.cpp"
  "${PDAL_SQLITE_SRC}/SQLiteWriter.cpp"
)

if (WITH_SQLITE)
    if (NOT USE_PDAL_PLUGIN_SQLITE)
        list (APPEND PDAL_CPP ${PDAL_DRIVERS_SQLITE_CPP} )
        list (APPEND PDAL_HPP ${PDAL_DRIVERS_SQLITE_HPP} )
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
  "${PDAL_PGPOINTCLOUD_HEADERS}/PgReader.hpp"
  "${PDAL_PGPOINTCLOUD_HEADERS}/PgIterator.hpp"
)

set (PDAL_DRIVERS_PGPOINTCLOUD_CPP
  "${PDAL_PGPOINTCLOUD_SRC}/Writer.cpp"
  "${PDAL_PGPOINTCLOUD_SRC}/PgReader.cpp"
  "${PDAL_PGPOINTCLOUD_SRC}/PgIterator.cpp"
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
# drivers/sbet
#
set(PDAL_SBET_PATH drivers/sbet)
set(PDAL_SBET_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_SBET_PATH}")
set(PDAL_SBET_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_SBET_PATH}")

set(PDAL_DRIVERS_SBET_HPP
    "${PDAL_SBET_HEADERS}/Common.hpp"
    "${PDAL_SBET_HEADERS}/Reader.hpp"
    "${PDAL_SBET_HEADERS}/Writer.hpp"
)

set (PDAL_DRIVERS_SBET_CPP
    "${PDAL_SBET_SRC}/Common.cpp"
    "${PDAL_SBET_SRC}/Reader.cpp"
    "${PDAL_SBET_SRC}/Writer.cpp"
)

list (APPEND PDAL_CPP ${PDAL_DRIVERS_SBET_CPP} )
list (APPEND PDAL_HPP ${PDAL_DRIVERS_SBET_HPP} )

#
# HDF5
#
set(PDAL_HDF5_HANDLER_HPP
    "${PDAL_HEADERS_DIR}/Hdf5Handler.hpp"
)

set(PDAL_HDF5_HANDLER_CPP
    "${PDAL_SOURCE_DIR}/src/Hdf5Handler.cpp"
)

if (HDF5_FOUND)
    list (APPEND PDAL_CPP ${PDAL_HDF5_HANDLER_CPP})
    list (APPEND PDAL_HPP ${PDAL_HDF5_HANDLER_HPP})
endif()


#
# drivers/icebridge
#
set(PDAL_ICEBRIDGE_PATH drivers/icebridge)
set(PDAL_ICEBRIDGE_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_ICEBRIDGE_PATH}")
set(PDAL_ICEBRIDGE_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_ICEBRIDGE_PATH}")

set(PDAL_DRIVERS_ICEBRIDGE_HPP
    "${PDAL_ICEBRIDGE_HEADERS}/Reader.hpp"
)

set(PDAL_DRIVERS_ICEBRIDGE_CPP
    "${PDAL_ICEBRIDGE_SRC}/Reader.cpp"
)

if (HDF5_FOUND)
    list (APPEND PDAL_CPP ${PDAL_DRIVERS_ICEBRIDGE_CPP} )
    list (APPEND PDAL_HPP ${PDAL_DRIVERS_ICEBRIDGE_HPP} )
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
  "${PDAL_FILTERS_HEADERS}/Crop.hpp"
  "${PDAL_FILTERS_HEADERS}/Decimation.hpp"
  "${PDAL_FILTERS_HEADERS}/HexBin.hpp"
  "${PDAL_FILTERS_HEADERS}/InPlaceReprojection.hpp"
  "${PDAL_FILTERS_HEADERS}/Merge.hpp"
  "${PDAL_FILTERS_HEADERS}/Reprojection.hpp"
  "${PDAL_FILTERS_HEADERS}/Scaling.hpp"
  "${PDAL_FILTERS_HEADERS}/Selector.hpp"
  "${PDAL_FILTERS_HEADERS}/Splitter.hpp"
  "${PDAL_FILTERS_HEADERS}/Stats.hpp"
)

set (PDAL_FILTERS_CPP
  "${PDAL_FILTERS_SRC}/Chipper.cpp"
  "${PDAL_FILTERS_SRC}/Crop.cpp"
  "${PDAL_FILTERS_SRC}/Decimation.cpp"
  "${PDAL_FILTERS_SRC}/HexBin.cpp"
  "${PDAL_FILTERS_SRC}/Reprojection.cpp"
  "${PDAL_FILTERS_SRC}/Splitter.cpp"
  "${PDAL_FILTERS_SRC}/Stats.cpp"
)

list (APPEND PDAL_CPP ${PDAL_FILTERS_CPP} )
list (APPEND PDAL_HPP ${PDAL_FILTERS_HPP} )

#
# Core components that depend on Python
#
set(PDAL_PYTHON_HPP
    ${PDAL_FILTERS_HEADERS}/Predicate.hpp
    ${PDAL_FILTERS_HEADERS}/Programmable.hpp
    )

set(PDAL_PYTHON_CPP
    ${PDAL_FILTERS_SRC}/Predicate.cpp
    ${PDAL_FILTERS_SRC}/Programmable.cpp
    )

if (WITH_PYTHON)
    list(APPEND PDAL_HPP ${PDAL_PYTHON_HPP})
    list(APPEND PDAL_CPP ${PDAL_PYTHON_CPP})
endif (WITH_PYTHON)


#
# Core components that depend on GDAL
#
set(PDAL_GDAL_HPP
    "${PDAL_HEADERS_DIR}/GDALUtils.hpp"
    "${PDAL_FILTERS_HEADERS}/Colorization.hpp"
    )

set(PDAL_GDAL_CPP
    GDALUtils.cpp
    "${PDAL_FILTERS_SRC}/Colorization.cpp"
    )

if (WITH_GDAL)
    list(APPEND PDAL_HPP ${PDAL_GDAL_HPP})
    list(APPEND PDAL_CPP ${PDAL_GDAL_CPP})
endif (WITH_GDAL)


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
    ${PDAL_KERNEL_HEADERS}/Diff.hpp
    ${PDAL_KERNEL_HEADERS}/Info.hpp
    ${PDAL_KERNEL_HEADERS}/Kernel.hpp
    ${PDAL_KERNEL_HEADERS}/Pipeline.hpp
    ${PDAL_KERNEL_HEADERS}/Delta.hpp
    ${PDAL_KERNEL_HEADERS}/Translate.hpp
)

set(PDAL_KERNEL_CPP
    ${PDAL_KERNEL_SRC}/Application.cpp
    ${PDAL_KERNEL_SRC}/Support.cpp
    ${PDAL_KERNEL_SRC}/Diff.cpp
    ${PDAL_KERNEL_SRC}/Info.cpp
    ${PDAL_KERNEL_SRC}/Pipeline.cpp
    ${PDAL_KERNEL_SRC}/Delta.cpp
    ${PDAL_KERNEL_SRC}/Translate.cpp
)

list (APPEND PDAL_CPP ${PDAL_KERNEL_CPP} )
list (APPEND PDAL_HPP ${PDAL_KERNEL_HPP} )


#
# pcl
#
set(PDAL_PCL_HPP
    ${PDAL_KERNEL_HEADERS}/PCL.hpp
    ${PDAL_FILTERS_HEADERS}/PCLBlock.hpp
    )
set(PDAL_PCL_CPP
    ${PDAL_FILTERS_SRC}/PCLBlock.cpp
    ${PDAL_KERNEL_SRC}/PCL.cpp
    )

if (WITH_PCL OR WITH_STUBS)
    list(APPEND PDAL_HPP ${PDAL_PCL_HPP})
    list(APPEND PDAL_CPP ${PDAL_PCL_CPP})
endif (WITH_PCL OR WITH_STUBS)


#
# Group source files for IDE source explorers (e.g. Visual Studio)
#

source_group("CMake Files" FILES CMakeLists.txt)

source_group("Header Files" FILES ${PDAL_BASE_HPP})
source_group("Header Files\\config" FILES ${PDAL_CONFIG_HPP})
source_group("Header Files\\drivers\\faux" FILES ${PDAL_DRIVERS_FAUX_HPP})
source_group("Header Files\\drivers\\caris" FILES ${PDAL_DRIVERS_CARIS_HPP})
source_group("Header Files\\drivers\\las" FILES ${PDAL_DRIVERS_LAS_HPP})
source_group("Header Files\\drivers\\nitf" FILES ${PDAL_DRIVERS_NITF_HPP})
source_group("Header Files\\drivers\\oci" FILES ${PDAL_DRIVERS_OCI_HPP})
source_group("Header Files\\drivers\\qfit" FILES ${PDAL_DRIVERS_QFIT_HPP})
source_group("Header Files\\drivers\\sbet" FILES ${PDAL_DRIVERS_SBET_HPP})
source_group("Header Files\\drivers\\greyhound" FILES ${PDAL_DRIVERS_GREYHOUND_HPP})
source_group("Header Files\\drivers\\terrasolid" FILES ${PDAL_DRIVERS_TERRASOLID_HPP})
source_group("Header Files\\drivers\\text" FILES ${PDAL_DRIVERS_TEXT_HPP})
source_group("Header Files\\drivers\\pgpointcloud" FILES ${PDAL_DRIVERS_PGPOINTCLOUD_HPP})
source_group("Header Files\\filters" FILES ${PDAL_FILTERS_HPP})
source_group("Header Files\\plang" FILES ${PDAL_PLANG_HPP})

source_group("Source Files" FILES ${PDAL_BASE_CPP})
source_group("Source Files\\config" FILES ${PDAL_CONFIG_CPP})
source_group("Source Files\\drivers\\faux" FILES ${PDAL_DRIVERS_FAUX_CPP})
source_group("Source Files\\drivers\\caris" FILES ${PDAL_DRIVERS_CARIS_CPP})
source_group("Source Files\\drivers\\las" FILES ${PDAL_DRIVERS_LAS_CPP})
source_group("Source Files\\drivers\\nitf" FILES ${PDAL_DRIVERS_NITF_CPP})
source_group("Source Files\\drivers\\oci" FILES ${PDAL_DRIVERS_OCI_CPP})
source_group("Source Files\\drivers\\qfit" FILES ${PDAL_DRIVERS_QFIT_CPP})
source_group("Source Files\\drivers\\sbet" FILES ${PDAL_DRIVERS_SBET_CPP})
source_group("Source Files\\drivers\\greyhound" FILES ${PDAL_DRIVERS_GREYHOUND_CPP})
source_group("Source Files\\drivers\\terrasolid" FILES ${PDAL_DRIVERS_TERRASOLID_CPP})
source_group("Source Files\\drivers\\text" FILES ${PDAL_DRIVERS_TEXT_CPP})
source_group("Source Files\\drivers\\pgpointcloud" FILES ${PDAL_DRIVERS_PGPOINTCLOUD_CPP})
source_group("Source Files\\filters" FILES ${PDAL_FILTERS_CPP})
source_group("Source Files\\plang" FILES ${PDAL_PLANG_CPP})


# Standard include directory of PDAL library
include_directories(../include)

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

PDAL_ADD_LIBRARY(${PDAL_LIB_NAME} ${PDAL_SOURCES})

set (SQLITE_LIBRARIES ${SQLITE3_LIBRARY})

if (WITH_SQLITE)
if (NOT USE_PDAL_PLUGIN_SQLITE)
target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE} ${SQLITE_LIBRARIES} )
endif()
endif()

if (WITH_PGPOINTCLOUD)
if (NOT USE_PDAL_PLUGIN_PGPOINTCLOUD)
target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE} ${POSTGRESQL_LIBRARIES})
endif()
endif()

if (WITH_ORACLE)
if (NOT USE_PDAL_PLUGIN_OCI)
target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE} "${ORACLE_LIBRARY}")
endif()
endif()

if (WITH_NITRO)
if (NOT USE_PDAL_PLUGIN_NITRO)
target_link_libraries(${PDAL_LIB_NAME}  ${PDAL_LINKAGE} ${NITRO_LIBRARIES} )
endif()
endif()

if (WITH_LASZIP)
    target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE} "${LASZIP_LIBRARY}")
endif()

if (WITH_PYTHON)
    target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE} "${PYTHON_LIBRARY}")
endif()

if (WITH_PCL)
    target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE} ${PCL_LIBRARIES})
endif()

if (WITH_LIBXML2)
    target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE} ${LIBXML2_LIBRARIES})
endif()

if (WITH_ZLIB)
    target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE} ${ZLIB_LIBRARIES})
endif()

if (WITH_HDF5)
    target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE} ${HDF5_LIBRARIES})
endif()

if (WITH_JSONCPP OR WITH_GREYHOUND)
    target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE} ${JSONCPP_LIBRARIES})
endif()

target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE}
    "${CMAKE_THREAD_LIBS_INIT}"
    "${CMAKE_DL_LIBS}"
    "${MRSID_LIBRARY}"
    "${GEOTIFF_LIBRARY}"
    "${GDAL_LIBRARY}"
    "${P2G_LIBRARY}"
    "${GEOS_LIBRARY}"
    "${HEXER_LIBRARY}"
    "${ORACLE_LIBRARY}"
)

message(STATUS "Using boost lib: ${Boost_LIBRARIES}")

target_link_libraries(${PDAL_LIB_NAME}
                       ${BOOST_LINKAGE} ${Boost_LIBRARIES})


set_target_properties(${PDAL_LIB_NAME}
  PROPERTIES SOVERSION "${PDAL_LIB_SOVERSION}" )


if (USE_PDAL_PLUGIN_TEXT)
    set(PDAL_TEXT_WRITER_LIB_NAME pdal_plugin_writer_text)
    add_library(${PDAL_TEXT_WRITER_LIB_NAME} SHARED "${PDAL_DRIVERS_TEXT_CPP}" "${PDAL_DRIVERS_TEXT_HPP}")
    target_link_libraries(${PDAL_TEXT_WRITER_LIB_NAME} INTERFACE general
                          ${PDAL_LIB_NAME})
    set_target_properties("${PDAL_TEXT_WRITER_LIB_NAME}"
    PROPERTIES SOVERSION "${PDAL_LIB_SOVERSION}" )
endif()

if (WITH_SQLITE)
if (USE_PDAL_PLUGIN_SQLITE)
    set(PDAL_SQLITE_WRITER_LIB_NAME pdal_plugin_writer_sqlite)
    add_library(${PDAL_SQLITE_WRITER_LIB_NAME} SHARED "${PDAL_SQLITE_SRC}/Writer.cpp")
    target_link_libraries(${PDAL_SQLITE_WRITER_LIB_NAME} INTERFACE general
                          "${PDAL_LIB_NAME}"
                          ${SQLITE_LIBRARIES})
    set_target_properties(${PDAL_SQLITE_WRITER_LIB_NAME}
    PROPERTIES SOVERSION "${PDAL_LIB_SOVERSION}" )

    set(PDAL_SQLITE_READER_LIB_NAME pdal_plugin_reader_sqlite)
    add_library(${PDAL_SQLITE_READER_LIB_NAME} SHARED ${PDAL_SQLITE_SRC}/Reader.cpp)
    target_link_libraries(${PDAL_SQLITE_READER_LIB_NAME} INTERFACE general
                          "${PDAL_LIB_NAME}"
                          ${SQLITE_LIBRARIES})
    set_target_properties(${PDAL_SQLITE_READER_LIB_NAME}
    PROPERTIES SOVERSION "${PDAL_LIB_SOVERSION}" )
endif()
endif()

if (WITH_ORACLE)
if (USE_PDAL_PLUGIN_OCI)
    set(PDAL_OCI_WRITER_LIB_NAME pdal_plugin_writer_oci)
    add_library(${PDAL_OCI_WRITER_LIB_NAME} SHARED "${PDAL_ORACLE_SRC}/oci_wrapper.cpp"
                                                    "${PDAL_ORACLE_SRC}/common.cpp"
                                                    "${PDAL_ORACLE_SRC}/Writer.cpp")
    target_link_libraries(${PDAL_OCI_WRITER_LIB_NAME} INTERFACE general
                          "${PDAL_LIB_NAME}"
                          "${ORACLE_LIBRARY}")
    set_target_properties(${PDAL_OCI_WRITER_LIB_NAME}
    PROPERTIES SOVERSION "${PDAL_LIB_SOVERSION}" )
endif()
endif()
###############################################################################
# Targets installation

install(DIRECTORY "${PDAL_HEADERS_DIR}/"
  DESTINATION "${PDAL_INCLUDE_DIR}"
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
