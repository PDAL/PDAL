#
# top-level CMake configuration file for PDAL
#
# (based originally on the libLAS files copyright Mateusz Loskot)

cmake_minimum_required(VERSION 3.5)

project(PDAL VERSION 2.1.0 LANGUAGES CXX C)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

set(ROOT_DIR "${PROJECT_SOURCE_DIR}")
include(${ROOT_DIR}/cmake/common.cmake NO_POLICY_SCOPE)

#------------------------------------------------------------------------------
# internal cmake settings
#------------------------------------------------------------------------------

set(CMAKE_COLOR_MAKEFILE ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
include(FeatureSummary)

# Allow advanced users to generate Makefiles printing detailed commands
mark_as_advanced(CMAKE_VERBOSE_MAKEFILE)

#------------------------------------------------------------------------------
# PDAL general settings
#------------------------------------------------------------------------------

set(PDAL_API_VERSION "9")
set(PDAL_BUILD_VERSION "10")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Name of C++ library

#
# To facilitate one-library linking, we do special things for various platforms.
#
# On Linux we install a linker script that we call libpdalcpp.so and it
# in turn links libpdal_base.so and libpdal_util.so
#
# On OSX we reexport the symbols from libpdal_util.dylib into libpdalcpp.dylib
# See below for the rest of the magic.
#
if (APPLE OR WIN32)
    set(PDAL_BASE_LIB_NAME pdalcpp)
else()
    set(PDAL_BASE_LIB_NAME pdal_base)
endif()
set(PDAL_LIB_NAME pdalcpp)
set(PDAL_UTIL_LIB_NAME pdal_util)
set(PDAL_BOOST_LIB_NAME pdal_boost)
set(PDAL_KAZHDAN_LIB_NAME pdal_kazhdan)
set(PDAL_TEST_SUPPORT_OBJS pdal_test_support)

set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

set(PDAL_OUTPUT_LIB_DIR "${PDAL_BINARY_DIR}/${PDAL_LIB_INSTALL_DIR}")
set(PDAL_OUTPUT_BIN_DIR "${PDAL_BINARY_DIR}/${PDAL_BIN_INSTALL_DIR}")

# allow override of PDAL_PLUGIN_INSTALL_PATH path
if (NOT PDAL_PLUGIN_INSTALL_PATH)
    if (WIN32)
        set(PDAL_PLUGIN_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/${PDAL_BIN_INSTALL_DIR}")
    else()
        set(PDAL_PLUGIN_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/${PDAL_LIB_INSTALL_DIR}")
    endif()
endif()
file(MAKE_DIRECTORY "${PDAL_OUTPUT_LIB_DIR}")
file(MAKE_DIRECTORY "${PDAL_OUTPUT_BIN_DIR}")

include(${PDAL_CMAKE_DIR}/rpath.cmake)

# wipe lib/ drectory on clean. It will have plugins that could be out of date
# in the next build
set_directory_properties(PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${PDAL_OUTPUT_LIB_DIR}/*")

if(WIN32)
  add_definitions("-DPDAL_DLL_EXPORT=1")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CMAKE_BUILD_TYPE} "${PDAL_OUTPUT_LIB_DIR}")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CMAKE_BUILD_TYPE} "${PDAL_OUTPUT_BIN_DIR}")
  # ---[ Windows requires DLLs (shared libraries) to be installed in
  # ---[ the same directory as executables
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CMAKE_BUILD_TYPE} "${PDAL_OUTPUT_BIN_DIR}")
endif(WIN32)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PDAL_OUTPUT_LIB_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PDAL_OUTPUT_BIN_DIR}")
if(WIN32)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PDAL_OUTPUT_BIN_DIR}")
else()
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PDAL_OUTPUT_LIB_DIR}")
endif()

# Choose package components

include(${PDAL_CMAKE_DIR}/options.cmake)

#------------------------------------------------------------------------------
# General build settings
#------------------------------------------------------------------------------

set(PDAL_BUILD_TYPE ${CMAKE_BUILD_TYPE})

#------------------------------------------------------------------------------
#  Dependencies.
#------------------------------------------------------------------------------

include(${PDAL_CMAKE_DIR}/gdal.cmake)
include(${PDAL_CMAKE_DIR}/geotiff.cmake)  # Optional (not really)
include(${PDAL_CMAKE_DIR}/lazperf.cmake)  # Optional
include(${PDAL_CMAKE_DIR}/laszip.cmake)  # Optional
include(${PDAL_CMAKE_DIR}/threads.cmake)
include(${PDAL_CMAKE_DIR}/zlib.cmake)
include(${PDAL_CMAKE_DIR}/lzma.cmake)
include(${PDAL_CMAKE_DIR}/zstd.cmake)
include(${PDAL_CMAKE_DIR}/test.cmake)
include(${PDAL_CMAKE_DIR}/ctest.cmake)
include(${PDAL_CMAKE_DIR}/libxml2.cmake)
include(${PDAL_CMAKE_DIR}/dimension.cmake)
include(${PDAL_CMAKE_DIR}/arbiter.cmake)
include(${PDAL_CMAKE_DIR}/nlohmann.cmake)
include(${PDAL_CMAKE_DIR}/openssl.cmake) # Optional

#------------------------------------------------------------------------------
# generate the pdal_features.hpp header
#------------------------------------------------------------------------------

# from http://stackoverflow.com/questions/1435953/how-can-i-pass-git-sha1-to-compiler-as-definition-using-cmake
include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC GIT_SHA1)

configure_file(
  "${PROJECT_SOURCE_DIR}/gitsha.cpp.in"
  "${PROJECT_SOURCE_DIR}/pdal/gitsha.cpp")

# needs to come before configuration of pdal_features
if(APPLE)
    option(PDAL_BUNDLE "Create PDAL as Application Bundle on OSX" FALSE)
    if (PDAL_BUNDLE)
        set(PDAL_APP_BUNDLE 1)
    endif()
endif()

set(pdal_features_hpp_in "${CMAKE_CURRENT_SOURCE_DIR}/pdal_features.hpp.in")
set(pdal_features_hpp
    "${CMAKE_CURRENT_BINARY_DIR}/include/pdal/pdal_features.hpp")
configure_file(${pdal_features_hpp_in} ${pdal_features_hpp})

#------------------------------------------------------------------------------
# subdirectory controls
#------------------------------------------------------------------------------

# PDAL_TARGET_OBJECTS is used to collect the driver object libraries
set(PDAL_TARGET_OBJECTS "")

if (WITH_TESTS)
    enable_testing()
endif()

add_subdirectory(plugins)

#include_directories(vendor/pdalboost)
if (WITH_TESTS)
    include (${PDAL_CMAKE_DIR}/gtest.cmake)
    add_subdirectory(test)
endif()
add_subdirectory(dimbuilder)
add_subdirectory(vendor/pdalboost)
add_subdirectory(vendor/arbiter)
add_subdirectory(vendor/kazhdan)
add_subdirectory(pdal/util)
add_subdirectory(tools)
add_subdirectory(apps)
#
# On OSX we reexport the symbols in libpdal_util.dylib into libpdalcpp.dylib
# so that users only need link libpdalcpp.
#
if (APPLE)
    set(PDAL_REEXPORT "-Wl,-reexport_library,$<TARGET_FILE:${PDAL_UTIL_LIB_NAME}>")
    #
    # This allows the rpath reference for the reexported library (above) to
    # be found.
    #
    set(PDAL_LIBDIR "-L$<TARGET_FILE_DIR:${PDAL_UTIL_LIB_NAME}>")
endif()

file(GLOB BASE_SRCS
    ${PDAL_FILTERS_DIR}/*.cpp
    ${PDAL_IO_DIR}/*.cpp
    ${PDAL_KERNELS_DIR}/*.cpp
    ${PDAL_SRC_DIR}/*.cpp
    ${PDAL_SRC_DIR}/compression/*.cpp)
file(GLOB_RECURSE PRIVATE_SRCS
    ${PDAL_FILTERS_DIR}/private/*.cpp
    ${PDAL_IO_DIR}/private/*.cpp
    ${PDAL_KERNELS_DIR}/private/*.cpp
    ${PDAL_SRC_DIR}/private/*.cpp)
list(APPEND SRCS ${BASE_SRCS} ${PRIVATE_SRCS})

#
# Remove stuff we don't want to build.
#
if (NOT PDAL_HAVE_LIBXML2)
    file(GLOB XML_SRCS
        io/Ilvis2MetadataReader.cpp
        io/Ilvis2Metadata.cpp
        io/Ilvis2Reader.cpp
        ${PDAL_SRC_DIR}/DbWriter.cpp
        ${PDAL_SRC_DIR}/DbReader.cpp
        ${PDAL_SRC_DIR}/XMLSchema.cpp)
    list(REMOVE_ITEM SRCS ${XML_SRCS})
endif()
if (NOT PDAL_HAVE_ZSTD)
    file(GLOB ZSTD_SRCS
        ${PDAL_SRC_DIR}/compression/ZstdCompression.cpp)
    list(REMOVE_ITEM SRCS ${ZSTD_SRCS})
endif()
if (NOT PDAL_HAVE_ZLIB)
    file(GLOB ZLIB_SRCS
        ${PDAL_SRC_DIR}/compression/DeflateCompression.cpp)
    list(REMOVE_ITEM SRCS ${ZLIB_SRCS})
endif()
if (NOT PDAL_HAVE_LZMA)
    file(GLOB LZMA_SRCS
        ${PDAL_SRC_DIR}/compression/LzmaCompression.cpp)
    list(REMOVE_ITEM SRCS ${LZMA_SRCS})
endif()
if (NOT PDAL_HAVE_LAZPERF)
    file(GLOB LAZPERF_SRCS
        ${PDAL_SRC_DIR}/compression/LazPerfCompression.cpp
        ${PDAL_SRC_DIR}/compression/LazPerfVlrCompression.cpp)
    list(REMOVE_ITEM SRCS ${LAZPERF_SRCS})
endif()
if (GDAL_VERSION VERSION_LESS 3.0.0)
    file(GLOB PROJPIPELINE_FILTER_SRCS
        ${PDAL_FILTERS_DIR}/ProjPipelineFilter.cpp)
    list(REMOVE_ITEM SRCS ${PROJPIPELINE_FILTER_SRCS})
endif()
PDAL_ADD_LIBRARY(${PDAL_BASE_LIB_NAME} ${SRCS} ${RPLY_SRCS})

#
# Interface include directories allow downstream project to get the directory
# without specification.
#
target_include_directories(${PDAL_BASE_LIB_NAME}
    PRIVATE
        ${ROOT_DIR}
        ${PROJECT_BINARY_DIR}/include
        ${PDAL_VENDOR_DIR}
        ${PDAL_VENDOR_DIR}/eigen
        ${PDAL_VENDOR_DIR}/pdalboost
        ${LIBXML2_INCLUDE_DIR}
        ${ZSTD_INCLUDE_DIRS}
        ${NLOHMANN_INCLUDE_DIR}
    PUBLIC
        ${GDAL_INCLUDE_DIR}
        ${LASZIP_INCLUDE_DIR}
)
target_link_libraries(${PDAL_BASE_LIB_NAME}
    PRIVATE
        ${CMAKE_THREAD_LIBS_INIT}
        ${GDAL_LIBRARY}
        ${GEOTIFF_LIBRARY}
        ${LASZIP_LIBRARY}
        ${LIBXML2_LIBRARIES}
        ${ZLIB_LIBRARIES}
        ${LIBLZMA_LIBRARIES}
        ${ZSTD_LIBRARIES}
        ${WINSOCK_LIBRARY}
        ${PDAL_REEXPORT}
        ${PDAL_UTIL_LIB_NAME}
        ${PDAL_ARBITER_LIB_NAME}
        ${PDAL_KAZHDAN_LIB_NAME}
    INTERFACE
        ${PDAL_LIBDIR}
)
target_compile_definitions(${PDAL_BASE_LIB_NAME}
    PRIVATE
        ${LASZIP_DEFINES}
)
set_target_properties(${PDAL_BASE_LIB_NAME} PROPERTIES
    VERSION ${PDAL_BUILD_VERSION}
    SOVERSION ${PDAL_API_VERSION}
    CLEAN_DIRECT_OUTPUT 1)

# shut off -Wpedantic selectively
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
set_source_files_properties(filters/PoissonFilter.cpp PROPERTIES COMPILE_FLAGS -Wno-pedantic)
endif()

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

#
# Installation
#

#
# Only install compression headers if we're building with the particular
# compression type.
#
if (NOT PDAL_HAVE_ZSTD)
    set(ZSTD_EXCLUDES PATTERN pdal/compression/Zstd* EXCLUDE)
endif()
if (NOT PDAL_HAVE_ZLIB)
    set(ZLIB_EXCLUDES PATTERN pdal/compression/Deflate* EXCLUDE)
endif()
if (NOT PDAL_HAVE_LZMA)
    set(LZMA_EXCLUDES PATTERN pdal/compression/Lzma* EXCLUDE)
endif()
if (NOT PDAL_HAVE_LAZPERF)
    set(LAZPERF_EXCLUDES PATTERN pdal/compression/LazPerf* EXCLUDE)
endif()

install(DIRECTORY ${PDAL_INCLUDE_DIR}/pdal
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN "gitsha.h"
    PATTERN "pdal/private" EXCLUDE
    PATTERN "pdal/io/private" EXCLUDE
    PATTERN "pdal/pdal/private" EXCLUDE
    PATTERN "pdal/util/private" EXCLUDE
    PATTERN "pdal/filters/private" EXCLUDE
    ${ZSTD_EXCLUDES}
    ${ZLIB_EXCLUDES}
    ${LZMA_EXCLUDES}
    ${LAZPERF_EXCLUDES}
)

install(DIRECTORY ${PDAL_KERNELS_DIR}
    DESTINATION include/pdal
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN "private" EXCLUDE
)
install(DIRECTORY ${PDAL_IO_DIR}
    DESTINATION include/pdal
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN "private" EXCLUDE
)
install(DIRECTORY ${PDAL_FILTERS_DIR}
    DESTINATION include/pdal
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN "private" EXCLUDE
)

install(FILES ${DIMENSION_OUTFILE} ${pdal_features_hpp}
  DESTINATION include/pdal
)

#
# CPACK
#
include (${PDAL_CMAKE_DIR}/cpack.cmake)

add_custom_target(dist COMMAND ${CMAKE_MAKE_PROGRAM} package_source)

export(
    TARGETS
        ${PDAL_BASE_LIB_NAME} ${PDAL_UTIL_LIB_NAME}
    FILE
        "${PDAL_BINARY_DIR}/PDALTargets.cmake")

install(
    EXPORT
        PDALTargets
    DESTINATION
        "${PDAL_LIB_INSTALL_DIR}/cmake/PDAL")
include(${PDAL_CMAKE_DIR}/config.cmake)
feature_summary(WHAT ALL INCLUDE_QUIET_PACKAGES)

# TODO: move under scripts/bash-completion ?
if (WITH_COMPLETION)
    if (IS_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/bash-completion/completions)
        install(
            FILES
                "${PROJECT_SOURCE_DIR}/scripts/bash-completion/pdal"
            DESTINATION
                "${CMAKE_INSTALL_PREFIX}/share/bash-completion/completions")
    elseif (IS_DIRECTORY /etc/bash_completion.d)
        install(
            FILES
                "${PROJECT_SOURCE_DIR}/scripts/bash-completion/pdal"
            DESTINATION
                "${CMAKE_INSTALL_PREFIX}/etc/bash_completion.d")
    endif()
endif()
