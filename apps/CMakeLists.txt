#
# ./apps CMake configuration file for PDAL utilties
#
# (based originally on the libLAS files copyright Mateusz Loskot)

cmake_minimum_required(VERSION 2.8)

#------------------------------------------------------------------------------
# Configure build targets
#------------------------------------------------------------------------------

set(PDAL_APP pdal)

if (APPLE AND PDAL_BUNDLE)
    add_executable(${PDAL_APP} MACOSX_BUNDLE pdal.cpp)
else (APPLE AND PDAL_BUNDLE)
    add_executable(${PDAL_APP} pdal.cpp)
endif(APPLE AND PDAL_BUNDLE)
pdal_target_compile_settings(${PDAL_APP})
target_link_libraries(${PDAL_APP}
    PRIVATE
        ${PDAL_BASE_LIB_NAME}
        ${PDAL_UTIL_LIB_NAME}
        ${GDAL_LIBRARY}
        ${WINSOCK_LIBRARY}
)
target_include_directories(${PDAL_APP} PRIVATE
    ${NLOHMANN_INCLUDE_DIR}
    ${PDAL_INCLUDE_DIR}
    ${GDAL_INCLUDE_DIR}
    ${PROJECT_BINARY_DIR}/include)

#------------------------------------------------------------------------------
# Targets installation
#------------------------------------------------------------------------------

if (APPLE AND PDAL_BUNDLE)
    install(TARGETS ${PDAL_APP}
        BUNDLE DESTINATION ${PDAL_BIN_INSTALL_DIR})
else(APPLE AND PDAL_BUNDLE)
    install(TARGETS ${PDAL_APP}
        RUNTIME DESTINATION ${PDAL_BIN_INSTALL_DIR})
endif(APPLE AND PDAL_BUNDLE)

set(PKGCONFIG_LIBRARY_DEFINITIONS "")
set(PDAL_INCLUDE_DEFINITIONS
    "-I${CMAKE_INSTALL_PREFIX}/include -I${GDAL_INCLUDE_DIR}")
set(PKGCONFIG_LIBRARY_DEFINITIONS "${PKGCONFIG_LIBRARY_DEFINITIONS} gdal")

if(LIBXML2_FOUND)
    set(PKGCONFIG_LIBRARY_DEFINITIONS
        "${PKGCONFIG_LIBRARY_DEFINITIONS} libxml-2.0")
    set(PDAL_INCLUDE_DEFINITIONS
        "${PDAL_INCLUDE_DEFINITIONS} -I${LIBXML2_INCLUDE_DIR}")
endif()

set(PDAL_INCLUDE_DEFINITIONS ${PDAL_INCLUDE_DEFINITIONS})

if (LASZIP_FOUND)
    set(PDAL_INCLUDE_DEFINITIONS
        "${PDAL_INCLUDE_DEFINITIONS} -I${LASZIP_INCLUDE_DIR}")
endif()

file(MAKE_DIRECTORY "${PDAL_OUTPUT_LIB_DIR}/pkgconfig/")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/pdal.pc.in
               ${CMAKE_CURRENT_BINARY_DIR}/pdal.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pdal.pc
    DESTINATION "${PDAL_LIB_INSTALL_DIR}/pkgconfig/"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/pdal-config.in"
               "${PDAL_OUTPUT_BIN_DIR}/pdal-config" @ONLY
               NEWLINE_STYLE UNIX)
install(PROGRAMS "${PDAL_OUTPUT_BIN_DIR}/pdal-config"
    DESTINATION
        "${CMAKE_INSTALL_PREFIX}/bin"
    PERMISSIONS
        OWNER_READ OWNER_WRITE OWNER_EXECUTE
        GROUP_READ GROUP_EXECUTE
        WORLD_READ WORLD_EXECUTE)

if(WIN32)
    # Autoconf compatibility variables to use the same script source.
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/pdal-config-bat.in"
                   "${PDAL_OUTPUT_BIN_DIR}/pdal-config.bat" @ONLY
                   NEWLINE_STYLE CRLF)
    install(PROGRAMS "${PDAL_OUTPUT_BIN_DIR}/pdal-config.bat"
        DESTINATION
            "${CMAKE_INSTALL_PREFIX}/bin"
        PERMISSIONS
            OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE
            WORLD_READ WORLD_EXECUTE)
endif()
