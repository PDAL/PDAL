#
# top-level CMake configuration file for PDAL
#
# (based originally on the libLAS files copyright Mateusz Loskot)

cmake_minimum_required(VERSION 3.13)

project(PDAL VERSION 2.9.0 LANGUAGES CXX C)
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

#
# Must be changed if there is an ABI change.  This builds the SONAME
# that's embedded in the library and any plugins.
include(${ROOT_DIR}/cmake/libraries.cmake NO_POLICY_SCOPE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#
# Name of C++ library
#
set(PDAL_LIB_NAME pdalcpp)

set(PDAL_KAZHDAN_LIB_NAME pdal_kazhdan)
set(PDAL_SPZ_LIB_NAME pdal_spz)
set(PDAL_LEPCC_LIB_NAME pdal_lepcc)
set(PDAL_TEST_SUPPORT_OBJS pdal_test_support)

set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

set(PDAL_OUTPUT_LIB_DIR "${PROJECT_BINARY_DIR}/lib")
set(PDAL_OUTPUT_BIN_DIR "${PROJECT_BINARY_DIR}/bin")


if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PDAL_OUTPUT_LIB_DIR}")
endif()

if (NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PDAL_OUTPUT_BIN_DIR}")
endif()

# allow override of PDAL_PLUGIN_INSTALL_PATH path
if (NOT PDAL_PLUGIN_INSTALL_PATH)
    if (WIN32)
        set(PDAL_PLUGIN_INSTALL_PATH "${CMAKE_INSTALL_BINDIR}" CACHE PATH "PDAL Plugin install location")
    else()
        set(PDAL_PLUGIN_INSTALL_PATH "${CMAKE_INSTALL_LIBDIR}" CACHE PATH "PDAL Plugin install location")
    endif()
endif()

include(${PDAL_CMAKE_DIR}/rpath.cmake)

# wipe lib/ drectory on clean. It will have plugins that could be out of date
# in the next build
set_directory_properties(PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${PDAL_OUTPUT_LIB_DIR}/*")

if(WIN32)
  add_definitions("-DPDAL_DLL_EXPORT=1")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CMAKE_BUILD_TYPE} "${PDAL_OUTPUT_LIB_DIR}" )
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CMAKE_BUILD_TYPE} "${PDAL_OUTPUT_BIN_DIR}" )
  # ---[ Windows requires DLLs (shared libraries) to be installed in
  # ---[ the same directory as executables
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CMAKE_BUILD_TYPE} "${PDAL_OUTPUT_BIN_DIR}" )
endif(WIN32)


# Choose package components

include(${PDAL_CMAKE_DIR}/options.cmake)

#------------------------------------------------------------------------------
# General build settings
#------------------------------------------------------------------------------

set(PDAL_BUILD_TYPE ${CMAKE_BUILD_TYPE})

#------------------------------------------------------------------------------
#  Dependencies.
#------------------------------------------------------------------------------

include(${PDAL_CMAKE_DIR}/backtrace.cmake)
include(${PDAL_CMAKE_DIR}/proj.cmake)
include(${PDAL_CMAKE_DIR}/gdal.cmake)
include(${PDAL_CMAKE_DIR}/geotiff.cmake)  # Optional (not really)
include(${PDAL_CMAKE_DIR}/threads.cmake)
include(${PDAL_CMAKE_DIR}/zlib.cmake)
include(${PDAL_CMAKE_DIR}/lzma.cmake)
include(${PDAL_CMAKE_DIR}/zstd.cmake)
include(${PDAL_CMAKE_DIR}/test.cmake)
include(${PDAL_CMAKE_DIR}/ctest.cmake)
include(${PDAL_CMAKE_DIR}/libxml2.cmake)
include(${PDAL_CMAKE_DIR}/dimension.cmake)
include(${PDAL_CMAKE_DIR}/arbiter.cmake)
include(${PDAL_CMAKE_DIR}/h3.cmake)
include(${PDAL_CMAKE_DIR}/nlohmann.cmake)
include(${PDAL_CMAKE_DIR}/schema-validator.cmake)
include(${PDAL_CMAKE_DIR}/lazperf.cmake)
include(${PDAL_CMAKE_DIR}/gcs.cmake) # Optional
include(${PDAL_CMAKE_DIR}/utfcpp.cmake)

#------------------------------------------------------------------------------
# generate the pdal_features.hpp header
#------------------------------------------------------------------------------

# from http://stackoverflow.com/questions/1435953/how-can-i-pass-git-sha1-to-compiler-as-definition-using-cmake
include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC GIT_SHA1)

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

if (WITH_TESTS)
    include (${PDAL_CMAKE_DIR}/gtest.cmake)
    add_subdirectory(test)
endif()
add_subdirectory(dimbuilder)
add_subdirectory(vendor/arbiter)
add_subdirectory(vendor/schema-validator)
add_subdirectory(vendor/kazhdan)
add_subdirectory(vendor/lazperf)
add_subdirectory(vendor/utfcpp)
add_subdirectory(vendor/lepcc)
add_subdirectory(vendor/h3)
add_subdirectory(vendor/spz)
add_subdirectory(tools)
add_subdirectory(apps)

file(GLOB BASE_SRCS
    ${PDAL_FILTERS_DIR}/*.cpp
    ${PDAL_IO_DIR}/*.cpp
    ${PDAL_KERNELS_DIR}/*.cpp
    ${PDAL_SRC_DIR}/*.cpp
    ${PDAL_SRC_DIR}/compression/*.cpp
    ${PDAL_SRC_DIR}/util/*.cpp)
file(GLOB_RECURSE PRIVATE_SRCS
    ${PDAL_FILTERS_DIR}/private/*.cpp
    ${PDAL_IO_DIR}/private/*.cpp
    ${PDAL_KERNELS_DIR}/private/*.cpp
    ${PDAL_SRC_DIR}/private/*.cpp
    ${BACKTRACE_SOURCE})
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

PDAL_ADD_LIBRARY(${PDAL_LIB_NAME} ${SRCS})
if (WIN32)
    # fix for "error C4005: '_XKEYCHECK_H': macro redefinition"
    # C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Tools\MSVC\14.29.30133\
    # include\xkeycheck.h(55): fatal error C1189: #error:  The C++ Standard Library forbids
    # macroizing the keyword "bool". Enable warning C4005 to find the forbidden define.
    target_compile_definitions(${PDAL_LIB_NAME} PRIVATE _XKEYCHECK_H=1)
endif()
add_library(PDAL::PDAL ALIAS ${PDAL_LIB_NAME})
#
# Interface include directories allow downstream project to get the directory
# without specification.
#



target_include_directories(${PDAL_LIB_NAME}
    SYSTEM BEFORE PRIVATE
        ${PDAL_VENDOR_EIGEN_DIR}
        ${PDAL_VENDOR_DIR}/kazhdan
        ${PDAL_VENDOR_DIR}/h3/include
)
target_include_directories(${PDAL_LIB_NAME}
    PRIVATE
        ${ROOT_DIR}
        ${PROJECT_BINARY_DIR}/include
        ${PDAL_VENDOR_DIR}
        ${LIBXML2_INCLUDE_DIR}
        ${ZSTD_INCLUDE_DIRS}
        ${NLOHMANN_INCLUDE_DIR}
        ${UTFCPP_INCLUDE_DIR}
        ${GDAL_INCLUDE_DIR}
        ${PROJ_INCLUDE_DIR}
)

target_link_libraries(${PDAL_LIB_NAME}
    PRIVATE
        ${CMAKE_THREAD_LIBS_INIT}
        ${CMAKE_DL_LIBS}
        ${GDAL_LIBRARY}
        ${PROJ_LIBRARIES}
        ${GEOTIFF_LIBRARY}
        ${LIBXML2_LIBRARIES}
        ${ZLIB_LIBRARIES}
        ${LIBLZMA_LIBRARIES}
        ${ZSTD_LIBRARIES}
        ${PDAL_REEXPORT}
        ${PDAL_ARBITER_LIB_NAME}
        ${PDAL_H3_LIB_NAME}
        ${JSON_SCHEMA_LIB_NAME}
        ${PDAL_KAZHDAN_LIB_NAME}
        ${PDAL_LEPCC_LIB_NAME}
        ${PDAL_LAZPERF_LIB_NAME}
        ${UTFCPP_LIB_NAME}
        ${BACKTRACE_LIBRARIES}
        ${PDAL_SPZ_LIB_NAME}
       "$<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:-lstdc++fs>"
    PUBLIC
        ${WINSOCK_LIBRARY}
    INTERFACE
        ${PDAL_LIBDIR}
        ${WINSOCK_LIBRARY}
)

#
# Allow downstream cmake projects to find PDAL header files without
# being explicit.
#
target_include_directories(${PDAL_LIB_NAME}
    INTERFACE
        $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PDAL_LIB_NAME}
    PRIVATE
    H3_PREFIX=PDALH3)

if(WIN32)
    target_compile_definitions(${PDAL_LIB_NAME}
        PUBLIC
            PDAL_DLL_EXPORT=1
    )
endif(WIN32)

set_target_properties(${PDAL_LIB_NAME} PROPERTIES
    VERSION ${PDAL_BUILD_VERSION}
    SOVERSION ${PDAL_API_VERSION}
    CLEAN_DIRECT_OUTPUT 1)

# shut off -Wpedantic selectively
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
    set_source_files_properties(filters/PoissonFilter.cpp PROPERTIES COMPILE_FLAGS "-Wno-pedantic -Wno-array-parameter")
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

install(DIRECTORY ${PDAL_INCLUDE_DIR}/pdal/
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/pdal"
    FILES_MATCHING PATTERN "*.hpp"
    PERMISSIONS
        GROUP_EXECUTE WORLD_EXECUTE OWNER_EXECUTE
        GROUP_READ WORLD_READ OWNER_READ
        OWNER_WRITE
#    PATTERN "pdal/private" EXCLUDE
#    PATTERN "pdal/io/private" EXCLUDE
#    PATTERN "pdal/pdal/private" EXCLUDE
#    PATTERN "pdal/util/private" EXCLUDE
#    PATTERN "pdal/filters/private" EXCLUDE
#    ${ZSTD_EXCLUDES}
#    ${ZLIB_EXCLUDES}
#    ${LZMA_EXCLUDES}
)

install(DIRECTORY ${PDAL_KERNELS_DIR}/
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/pdal/kernels"
    FILES_MATCHING PATTERN "*.hpp"
    PERMISSIONS
        GROUP_EXECUTE WORLD_EXECUTE OWNER_EXECUTE
        GROUP_READ WORLD_READ OWNER_READ
        OWNER_WRITE
#    PATTERN "private" EXCLUDE
)
install(DIRECTORY ${PDAL_IO_DIR}/
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/pdal/io"
    FILES_MATCHING PATTERN "*.hpp"
    PERMISSIONS
        GROUP_EXECUTE WORLD_EXECUTE OWNER_EXECUTE
        GROUP_READ WORLD_READ OWNER_READ
        OWNER_WRITE
#    PATTERN "private" EXCLUDE
)
install(DIRECTORY ${PDAL_FILTERS_DIR}/
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/pdal/filters"
    FILES_MATCHING PATTERN "*.hpp"
    PERMISSIONS
        GROUP_EXECUTE WORLD_EXECUTE OWNER_EXECUTE
        GROUP_READ WORLD_READ OWNER_READ
        OWNER_WRITE
#    PATTERN "private" EXCLUDE
)

install(FILES ${DIMENSION_OUTFILE} ${pdal_features_hpp}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/pdal"
    PERMISSIONS
        GROUP_EXECUTE WORLD_EXECUTE OWNER_EXECUTE
        GROUP_READ WORLD_READ OWNER_READ
        OWNER_WRITE
)
install(FILES ${PDAL_CMAKE_DIR}/pluginmacros.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PDAL
    PERMISSIONS
        GROUP_EXECUTE WORLD_EXECUTE OWNER_EXECUTE
        GROUP_READ WORLD_READ OWNER_READ
        OWNER_WRITE
)

#
# CPACK
#
include (${PDAL_CMAKE_DIR}/cpack.cmake)

add_custom_target(dist COMMAND ${CMAKE_MAKE_PROGRAM} package_source)

export(
    TARGETS
        ${PDAL_LIB_NAME}
    FILE
        "${PDAL_BINARY_DIR}/PDALTargets.cmake")

install(
    EXPORT
        PDALTargets
    DESTINATION
        "${CMAKE_INSTALL_LIBDIR}/cmake/PDAL")
include(${PDAL_CMAKE_DIR}/config.cmake)



feature_summary(WHAT ALL INCLUDE_QUIET_PACKAGES)

# TODO: move under scripts/bash-completion ?
if (WITH_COMPLETION)
    if (IS_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/bash-completion/completions)
        install(
            FILES
                "${PROJECT_SOURCE_DIR}/scripts/bash-completion/pdal"
            DESTINATION
                "${CMAKE_INSTALL_DATAROOTDIR}/bash-completion/completions")
    elseif (IS_DIRECTORY /etc/bash_completion.d)
        install(
            FILES
                "${PROJECT_SOURCE_DIR}/scripts/bash-completion/pdal"
            DESTINATION
                "${CMAKE_INSTALL_SYSCONFDIR}/bash_completion.d")
    endif()
endif()
