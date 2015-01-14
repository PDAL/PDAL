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
set(PDAL_UTIL_DIR "${PROJECT_SOURCE_DIR}/util")
set(PDAL_HPP "")
set(PDAL_CPP "")

if (PDAL_HAVE_LIBXML2)
    set(PDAL_XML_HEADER ${PDAL_HEADERS_DIR}/XMLSchema.hpp)
    set(DB_DRIVER_HEADERS
        "${PDAL_HEADERS_DIR}/DbReader.hpp"
        "${PDAL_HEADERS_DIR}/DbWriter.hpp"
    )
    set(PDAL_XML_SRC XMLSchema.cpp)
    set(DB_DRIVER_SRCS
        DbReader.cpp
        DbWriter.cpp
    )
endif()

#
# base
#
set(PDAL_BASE_HPP
  "${PDAL_HEADERS_DIR}/pdal_error.hpp"
  "${PDAL_HEADERS_DIR}/pdal_types.hpp"
  "${PDAL_HEADERS_DIR}/Charbuf.hpp"
  "${PDAL_HEADERS_DIR}/Compression.hpp"
  "${PDAL_HEADERS_DIR}/Dimension.hpp"
  "${PDAL_HEADERS_DIR}/Drivers.hpp"
  "${PDAL_HEADERS_DIR}/FileUtils.hpp"
  "${PDAL_HEADERS_DIR}/Filter.hpp"
  "${PDAL_HEADERS_DIR}/Filters.hpp"
  "${PDAL_HEADERS_DIR}/GDALUtils.hpp"
  "${PDAL_HEADERS_DIR}/GlobalEnvironment.hpp"
  "${PDAL_HEADERS_DIR}/gitsha.h"
  "${PDAL_HEADERS_DIR}/IStream.hpp"
  "${PDAL_HEADERS_DIR}/KDIndex.hpp"
  "${PDAL_HEADERS_DIR}/Log.hpp"
  "${PDAL_HEADERS_DIR}/Metadata.hpp"
  "${PDAL_HEADERS_DIR}/Options.hpp"
  "${PDAL_HEADERS_DIR}/OStream.hpp"
  "${PDAL_HEADERS_DIR}/PipelineManager.hpp"
  "${PDAL_HEADERS_DIR}/PipelineReader.hpp"
  "${PDAL_HEADERS_DIR}/PipelineWriter.hpp"
  "${PDAL_HEADERS_DIR}/PointBuffer.hpp"
  "${PDAL_HEADERS_DIR}/PointBufferIter.hpp"
  "${PDAL_HEADERS_DIR}/PointContext.hpp"
  "${PDAL_HEADERS_DIR}/QuadIndex.hpp"
  "${PDAL_HEADERS_DIR}/RawPtBuf.hpp"
  "${PDAL_HEADERS_DIR}/Reader.hpp"
  "${PDAL_HEADERS_DIR}/SpatialReference.hpp"
  "${PDAL_HEADERS_DIR}/Stage.hpp"
  "${PDAL_HEADERS_DIR}/StageFactory.hpp"
  "${PDAL_HEADERS_DIR}/StageInfo.hpp"
  "${PDAL_HEADERS_DIR}/StageRunner.hpp"
  "${PDAL_HEADERS_DIR}/StreamFactory.hpp"
  "${PDAL_HEADERS_DIR}/UserCallback.hpp"
  "${PDAL_HEADERS_DIR}/Utils.hpp"
  "${PDAL_HEADERS_DIR}/Writer.hpp"
  "${PDAL_HEADERS_DIR}/Kernels.hpp"
    ${PDAL_XML_HEADER}
    ${DB_DRIVER_HEADERS}
)

set(PDAL_BASE_CPP
  Charbuf.cpp
  FileUtils.cpp
  Filter.cpp
  gitsha.cpp
  GDALUtils.cpp
  GlobalEnvironment.cpp
  KDIndex.cpp
  Log.cpp
  Options.cpp
  PDALUtils.cpp
  PipelineManager.cpp
  PipelineReader.cpp
  PipelineWriter.cpp
  PointBuffer.cpp
  QuadIndex.cpp
  Reader.cpp
  SpatialReference.cpp
  Stage.cpp
  StageFactory.cpp
  StageInfo.cpp
  StreamFactory.cpp
  Utils.cpp
  Writer.cpp
  ${PDAL_XML_SRC}
  ${PDAL_LAZPERF_SRC}
  ${DB_DRIVER_SRCS}
)

list (APPEND PDAL_CPP ${PDAL_BASE_CPP} )
list (APPEND PDAL_HPP ${PDAL_BASE_HPP} )


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


# Standard include directory of PDAL library

###############################################################################
# Targets settings

set(PDAL_SOURCES
  ${PDAL_HPP}
  ${PDAL_CPP}
  ${PDAL_TARGET_OBJECTS}
)

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
    if (NOT WITH_STATIC_LASZIP)
        add_definitions("-DLASZIP_DLL_IMPORT=1")
    endif()
endif()

PDAL_ADD_LIBRARY(${PDAL_LIB_NAME} ${PDAL_SOURCES})

if (WITH_LASZIP)
    target_link_libraries(${PDAL_LIB_NAME} ${LASZIP_LIBRARY})
endif()

if (PDAL_HAVE_LIBXML2)
    target_link_libraries(${PDAL_LIB_NAME} ${LIBXML2_LIBRARIES})
endif()

target_link_libraries(${PDAL_LIB_NAME}
    ${CMAKE_THREAD_LIBS_INIT}
    ${GEOTIFF_LIBRARY}
    ${GDAL_LIBRARY}
    ${GEOS_LIBRARY}
    ${ZLIB_LIBRARIES}
    ${PDAL_UTIL_LIB_NAME}
)

target_link_libraries(${PDAL_LIB_NAME} ${Boost_LIBRARIES})

set_target_properties(${PDAL_LIB_NAME}
  PROPERTIES SOVERSION "${PDAL_LIB_SOVERSION}" )

###############################################################################
# Targets installation

install(DIRECTORY "${PDAL_HEADERS_DIR}/"
  DESTINATION "${PDAL_INCLUDE_INSTALL_DIR}"
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
