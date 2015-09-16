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
  "${PDAL_HEADERS_DIR}/pdal_types.hpp"
  "${PDAL_HEADERS_DIR}/BufferReader.hpp"
  "${PDAL_HEADERS_DIR}/Compression.hpp"
  "${PDAL_HEADERS_DIR}/Dimension.hpp"
  "${PDAL_HEADERS_DIR}/Filter.hpp"
  "${PDAL_HEADERS_DIR}/FlexWriter.hpp"
  "${PDAL_HEADERS_DIR}/GDALUtils.hpp"
  "${PDAL_HEADERS_DIR}/GlobalEnvironment.hpp"
  "${PDAL_HEADERS_DIR}/gitsha.h"
  "${PDAL_HEADERS_DIR}/KDIndex.hpp"
  "${PDAL_HEADERS_DIR}/KernelFactory.hpp"
  "${PDAL_HEADERS_DIR}/Kernel.hpp"
  "${PDAL_HEADERS_DIR}/KernelSupport.hpp"
  "${PDAL_HEADERS_DIR}/Log.hpp"
  "${PDAL_HEADERS_DIR}/Metadata.hpp"
  "${PDAL_HEADERS_DIR}/Options.hpp"
  "${PDAL_HEADERS_DIR}/PipelineManager.hpp"
  "${PDAL_HEADERS_DIR}/PipelineReader.hpp"
  "${PDAL_HEADERS_DIR}/PipelineWriter.hpp"
  "${PDAL_HEADERS_DIR}/PointLayout.hpp"
  "${PDAL_HEADERS_DIR}/PointTable.hpp"
  "${PDAL_HEADERS_DIR}/PointView.hpp"
  "${PDAL_HEADERS_DIR}/PointViewIter.hpp"
  "${PDAL_HEADERS_DIR}/QuadIndex.hpp"
  "${PDAL_HEADERS_DIR}/Reader.hpp"
  "${PDAL_HEADERS_DIR}/SpatialReference.hpp"
  "${PDAL_HEADERS_DIR}/Stage.hpp"
  "${PDAL_HEADERS_DIR}/StageFactory.hpp"
  "${PDAL_HEADERS_DIR}/StageWrapper.hpp"
  "${PDAL_HEADERS_DIR}/UserCallback.hpp"
  "${PDAL_HEADERS_DIR}/Writer.hpp"
  "${PDAL_SRC_DIR}/StageRunner.hpp"
    ${PDAL_XML_HEADER}
    ${DB_DRIVER_HEADERS}
)

set(PDAL_BASE_CPP
  DynamicLibrary.cpp
  Filter.cpp
  gitsha.cpp
  GDALUtils.cpp
  GlobalEnvironment.cpp
  Kernel.cpp
  KernelFactory.cpp
  KernelSupport.cpp
  Log.cpp
  Options.cpp
  PDALUtils.cpp

  PointLayout.cpp
  PointTable.cpp
  PointView.cpp

  PipelineManager.cpp
  PipelineReader.cpp
  PipelineWriter.cpp
  PluginManager.cpp
  QuadIndex.cpp
  Reader.cpp
  SpatialReference.cpp
  Stage.cpp
  StageFactory.cpp
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

PDAL_ADD_LIBRARY(${PDAL_BASE_LIB_NAME} ${PDAL_SOURCES})

set_target_properties(${PDAL_BASE_LIB_NAME} PROPERTIES VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)

if (WITH_LASZIP)
    target_link_libraries(${PDAL_BASE_LIB_NAME} ${LASZIP_LIBRARY})
endif()

if (PDAL_HAVE_LIBXML2)
    target_link_libraries(${PDAL_BASE_LIB_NAME} ${LIBXML2_LIBRARIES})
endif()

#
# On OSX we reexport the symbols in libpdal_util.dylib into libpdalcpp.dylib
# so that users only need link libpdalcpp.
#
if (APPLE)
    set(UTIL_LIB "-Wl,-reexport_library,$<TARGET_SONAME_FILE:${PDAL_UTIL_LIB_NAME}>")
else()
    set(UTIL_LIB ${PDAL_UTIL_LIB_NAME})
endif()


target_link_libraries(${PDAL_BASE_LIB_NAME}
    ${CMAKE_THREAD_LIBS_INIT}
    ${GDAL_LIBRARY}
    ${GEOS_LIBRARY}
    ${ZLIB_LIBRARIES}
    ${UTIL_LIB}
)

if (WITH_GEOTIFF)
    target_link_libraries(${PDAL_BASE_LIB_NAME} ${GEOTIFF_LIBRARY})
endif()

target_link_libraries(${PDAL_BASE_LIB_NAME} ${Boost_LIBRARIES})

if(WIN32)
    target_link_libraries(${PDAL_BASE_LIB_NAME} ws2_32)
endif()

###############################################################################
# Targets installation

#
# On Linux, we install a linker script as libpdalcpp.so.  That file
# specifies linking in libpdal_base.so and libpdal_util.so.  This allows
# users to link a single library, libpdalcpp
#
if (UNIX AND NOT APPLE)
    set(LIBNAME ${CMAKE_SHARED_LIBRARY_PREFIX}${PDAL_LIB_NAME})
    install(FILES ${LIBNAME} DESTINATION ${PDAL_LIB_INSTALL_DIR}
        RENAME ${LIBNAME}${CMAKE_SHARED_LIBRARY_SUFFIX})
endif()

install(DIRECTORY "${PDAL_HEADERS_DIR}/"
  DESTINATION "${PDAL_INCLUDE_INSTALL_DIR}"
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
