#
# ./apps CMake configuration file for PDAL utilties
#
# (based originally on the libLAS files copyright Mateusz Loskot)

cmake_minimum_required(VERSION 2.8)

set(PDAL_SOURCE_DIR ../src)

#------------------------------------------------------------------------------
# includes
#------------------------------------------------------------------------------

include_directories(
    .
    ../include
    ../util
    ${PROJECT_SOURCE_DIR}/io/faux
    ${PROJECT_SOURCE_DIR}/io/las
    ${PROJECT_SOURCE_DIR}/kernels
    ${PROJECT_SOURCE_DIR}/kernels/delta
    ${PROJECT_SOURCE_DIR}/kernels/diff
    ${PROJECT_SOURCE_DIR}/kernels/info
    ${PROJECT_SOURCE_DIR}/kernels/pipeline
    ${PROJECT_SOURCE_DIR}/kernels/random
    ${PROJECT_SOURCE_DIR}/kernels/sort
    ${PROJECT_SOURCE_DIR}/kernels/translate
)

#------------------------------------------------------------------------------
# Collect programs to build
#------------------------------------------------------------------------------

set(PDAL_UTILITY pdal)

#------------------------------------------------------------------------------
# Configure build targets
#------------------------------------------------------------------------------

link_directories(${Boost_LIBRARY_DIRS})

if(PDAL_UTILITY)
    set(srcs pdal.cpp)

    if(WIN32)
        list(APPEND srcs ${PDAL_TARGET_OBJECTS})
    endif()

    list(APPEND PDAL_UTILITIES ${PDAL_UTILITY})
    if (APPLE AND PDAL_BUNDLE)
        add_executable(${PDAL_UTILITY} MACOSX_BUNDLE ${srcs})
    else (APPLE AND PDAL_BUNDLE)
        add_executable(${PDAL_UTILITY} ${srcs})
    endif(APPLE AND PDAL_BUNDLE)
    target_link_libraries(${PDAL_UTILITY} ${PDAL_BASE_LIB_NAME})
endif()

#------------------------------------------------------------------------------
# Targets installation
#------------------------------------------------------------------------------

if (APPLE AND PDAL_BUNDLE)
    install(TARGETS ${PDAL_UTILITIES}
        BUNDLE DESTINATION ${PDAL_BIN_INSTALL_DIR})
else(APPLE AND PDAL_BUNDLE)
    install(TARGETS ${PDAL_UTILITIES}
        RUNTIME DESTINATION ${PDAL_BIN_INSTALL_DIR})
endif(APPLE AND PDAL_BUNDLE)

if(UNIX OR APPLE)

    set(PKGCONFIG_LIBRARY_DEFINITIONS "")
    set(PDAL_INCLUDE_DEFINITIONS "-I${CMAKE_INSTALL_PREFIX}/include -I${Boost_INCLUDE_DIR} -I${GDAL_INCLUDE_DIR}")
    set(PKGCONFIG_LIBRARY_DEFINITIONS "${PKGCONFIG_LIBRARY_DEFINITIONS} gdal")

    if(LIBXML2_FOUND)
        set(PKGCONFIG_LIBRARY_DEFINITIONS "${PKGCONFIG_LIBRARY_DEFINITIONS} libxml-2.0")
        set(PDAL_INCLUDE_DEFINITIONS "${PDAL_INCLUDE_DEFINITIONS} -I${LIBXML2_INCLUDE_DIR}")
    endif()

    if (GEOS_FOUND)
        set(PKGCONFIG_LIBRARY_DEFINITIONS "${PKGCONFIG_LIBRARY_DEFINITIONS} geos")
        set(PDAL_INCLUDE_DEFINITIONS "${PDAL_INCLUDE_DEFINITIONS} -I${GEOS_INCLUDE_DIR}")
    endif()

    if (LASZIP_FOUND)
        set(PDAL_INCLUDE_DEFINITIONS "${PDAL_INCLUDE_DEFINITIONS} -I${LASZIP_INCLUDE_DIR}")
    endif()

    file(MAKE_DIRECTORY "${PDAL_OUTPUT_LIB_DIR}/pkgconfig/")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/pdal.pc.in
                   ${CMAKE_CURRENT_BINARY_DIR}/pdal.pc @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pdal.pc
        DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/"
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)


    # Autoconf compatibility variables to use the same script source.
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/pdal-config.in"
                   "${CMAKE_CURRENT_SOURCE_DIR}/pdal-config" @ONLY)

    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/pdal-config"
         DESTINATION "${PDAL_OUTPUT_BIN_DIR}/"
         FILE_PERMISSIONS  OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

    install(PROGRAMS "${PDAL_OUTPUT_BIN_DIR}/pdal-config"
            DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
            PERMISSIONS
            OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endif()
