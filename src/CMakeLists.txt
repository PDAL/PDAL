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

set(PDAL_KERNEL_PATH kernel)
set(PDAL_KERNEL_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_KERNEL_PATH}")
set(PDAL_KERNEL_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_KERNEL_PATH}")

#
# base
#
set(PDAL_BASE_HPP
  "${PDAL_HEADERS_DIR}/pdal_error.hpp"
  "${PDAL_HEADERS_DIR}/pdal_types.hpp"
  "${PDAL_HEADERS_DIR}/Bounds.hpp"
  "${PDAL_HEADERS_DIR}/Charbuf.hpp"
  "${PDAL_HEADERS_DIR}/Compression.hpp"
  "${PDAL_HEADERS_DIR}/Dimension.hpp"
  "${PDAL_HEADERS_DIR}/Drivers.hpp"
  "${PDAL_HEADERS_DIR}/FileUtils.hpp"
  "${PDAL_HEADERS_DIR}/Filter.hpp"
  "${PDAL_HEADERS_DIR}/Filters.hpp"
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
  "${PDAL_HEADERS_DIR}/XMLSchema.hpp"
  "${PDAL_HEADERS_DIR}/third/nanoflann.hpp"
  "${PDAL_HEADERS_DIR}/third/md5-pdal.h"
  "${PDAL_HEADERS_DIR}/third/string_ref.hpp"
  "${PDAL_HEADERS_DIR}/third/string_ref_fwd.hpp"
    ${PDAL_KERNEL_HEADERS}/Kernel.hpp
    ${PDAL_KERNEL_HEADERS}/KernelFactory.hpp
    ${PDAL_KERNEL_HEADERS}/KernelInfo.hpp
    ${PDAL_KERNEL_HEADERS}/Kernels.hpp
    ${PDAL_KERNEL_HEADERS}/Support.hpp
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
  third/md5-pdal.c
  ${PDAL_XML_SRC}
  ${PDAL_LAZPERF_SRC}
    ${PDAL_KERNEL_SRC}/Kernel.cpp
    ${PDAL_KERNEL_SRC}/KernelFactory.cpp
    ${PDAL_KERNEL_SRC}/KernelInfo.cpp
    ${PDAL_KERNEL_SRC}/Support.cpp
)

list (APPEND PDAL_CPP ${PDAL_BASE_CPP} )
list (APPEND PDAL_HPP ${PDAL_BASE_HPP} )

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
# filters
#
set(PDAL_FILTERS_PATH filters)
set(PDAL_FILTERS_HEADERS "${PDAL_HEADERS_DIR}/${PDAL_FILTERS_PATH}")
set(PDAL_FILTERS_SRC "${PROJECT_SOURCE_DIR}/src/${PDAL_FILTERS_PATH}")

set(PDAL_FILTERS_HPP
  "${PDAL_FILTERS_HEADERS}/Chipper.hpp"
  "${PDAL_FILTERS_HEADERS}/Crop.hpp"
  "${PDAL_FILTERS_HEADERS}/Decimation.hpp"
  "${PDAL_FILTERS_HEADERS}/Ferry.hpp"
  "${PDAL_FILTERS_HEADERS}/Merge.hpp"
  "${PDAL_FILTERS_HEADERS}/MortonOrder.hpp"
  "${PDAL_FILTERS_HEADERS}/Reprojection.hpp"
  "${PDAL_FILTERS_HEADERS}/Splitter.hpp"
  "${PDAL_FILTERS_HEADERS}/Stats.hpp"
)

set (PDAL_FILTERS_CPP
  "${PDAL_FILTERS_SRC}/Chipper.cpp"
  "${PDAL_FILTERS_SRC}/Crop.cpp"
  "${PDAL_FILTERS_SRC}/Decimation.cpp"
  "${PDAL_FILTERS_SRC}/Ferry.cpp"
  "${PDAL_FILTERS_SRC}/MortonOrder.cpp"
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

list(APPEND PDAL_HPP ${PDAL_GDAL_HPP})
list(APPEND PDAL_CPP ${PDAL_GDAL_CPP})


#
# plang parser
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

set(PDAL_KERNEL_HPP
    ${PDAL_KERNEL_HEADERS}/Diff.hpp
    ${PDAL_KERNEL_HEADERS}/Info.hpp
    ${PDAL_KERNEL_HEADERS}/Pipeline.hpp
    ${PDAL_KERNEL_HEADERS}/Delta.hpp
    ${PDAL_KERNEL_HEADERS}/Random.hpp
    ${PDAL_KERNEL_HEADERS}/Sort.hpp
    ${PDAL_KERNEL_HEADERS}/Translate.hpp
)

set(PDAL_KERNEL_CPP
    ${PDAL_KERNEL_SRC}/Diff.cpp
    ${PDAL_KERNEL_SRC}/Info.cpp
    ${PDAL_KERNEL_SRC}/Pipeline.cpp
    ${PDAL_KERNEL_SRC}/Delta.cpp
    ${PDAL_KERNEL_SRC}/Random.cpp
    ${PDAL_KERNEL_SRC}/Sort.cpp
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
source_group("Header Files\\drivers\\las" FILES ${PDAL_DRIVERS_LAS_HPP})
source_group("Header Files\\drivers\\terrasolid" FILES ${PDAL_DRIVERS_TERRASOLID_HPP})
source_group("Header Files\\drivers\\text" FILES ${PDAL_DRIVERS_TEXT_HPP})
source_group("Header Files\\filters" FILES ${PDAL_FILTERS_HPP})
source_group("Header Files\\plang" FILES ${PDAL_PLANG_HPP})

source_group("Source Files" FILES ${PDAL_BASE_CPP})
source_group("Source Files\\config" FILES ${PDAL_CONFIG_CPP})
source_group("Source Files\\drivers\\las" FILES ${PDAL_DRIVERS_LAS_CPP})
source_group("Source Files\\drivers\\terrasolid" FILES ${PDAL_DRIVERS_TERRASOLID_CPP})
source_group("Source Files\\drivers\\text" FILES ${PDAL_DRIVERS_TEXT_CPP})
source_group("Source Files\\filters" FILES ${PDAL_FILTERS_CPP})
source_group("Source Files\\plang" FILES ${PDAL_PLANG_CPP})


# Standard include directory of PDAL library
include_directories(../include)

###############################################################################
# Targets settings

include_directories(${PROJECT_SOURCE_DIR}/drivers/bpf)
include_directories(${PROJECT_SOURCE_DIR}/drivers/buffer)
include_directories(${PROJECT_SOURCE_DIR}/drivers/faux)
include_directories(${PROJECT_SOURCE_DIR}/drivers/las)
include_directories(${PROJECT_SOURCE_DIR}/drivers/qfit)
include_directories(${PROJECT_SOURCE_DIR}/drivers/sbet)
include_directories(${PROJECT_SOURCE_DIR}/drivers/text)
include_directories(${PROJECT_SOURCE_DIR}/filters/attribute)

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
    add_definitions("-DPDAL_DLL_EXPORT=1")
    if (NOT WITH_STATIC_LASZIP)
        add_definitions("-DLASZIP_DLL_IMPORT=1")
    endif()
endif()

PDAL_ADD_LIBRARY(${PDAL_LIB_NAME} ${PDAL_SOURCES})

if (WITH_LASZIP)
    target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE} "${LASZIP_LIBRARY}")
endif()

if (WITH_PYTHON)
    target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE} "${PYTHON_LIBRARY}")
endif()

if (WITH_LIBXML2)
    target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE} ${LIBXML2_LIBRARIES})
endif()

target_link_libraries(${PDAL_LIB_NAME} ${PDAL_LINKAGE}
    "${CMAKE_THREAD_LIBS_INIT}"
    "${CMAKE_DL_LIBS}"
    "${GEOTIFF_LIBRARY}"
    "${GDAL_LIBRARY}"
    "${GEOS_LIBRARY}"
    "${ZLIB_LIBRARIES}"
)

message(STATUS "Using boost lib: ${Boost_LIBRARIES}")

target_link_libraries(${PDAL_LIB_NAME}
                       ${BOOST_LINKAGE} ${Boost_LIBRARIES})


set_target_properties(${PDAL_LIB_NAME}
  PROPERTIES SOVERSION "${PDAL_LIB_SOVERSION}" )


###############################################################################
# Targets installation

install(DIRECTORY "${PDAL_HEADERS_DIR}/"
  DESTINATION "${PDAL_INCLUDE_DIR}"
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
