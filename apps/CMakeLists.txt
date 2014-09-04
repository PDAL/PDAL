#
# ./apps CMake configuration file for PDAL utilties
#
# (based originally on the libLAS files copyright Mateusz Loskot)

cmake_minimum_required(VERSION 2.8)

if(FREEGLUT_FOUND)
  subdirs(pcview)
endif()

set(PDAL_SOURCE_DIR ../src)

#------------------------------------------------------------------------------
# includes
#------------------------------------------------------------------------------

include_directories(
    .
    ../include)

#------------------------------------------------------------------------------
# Collect programs to build
#------------------------------------------------------------------------------

set(PDAL_UTILITY pdal)


#------------------------------------------------------------------------------
# Configure build targets
#------------------------------------------------------------------------------

if(WIN32)
    add_definitions("-DPDAL_DLL_EXPORT=1")
endif()

link_directories(${Boost_LIBRARY_DIRS})

if(PDAL_UTILITY)
    list(APPEND PDAL_UTILITIES ${PDAL_UTILITY})
    add_executable(${PDAL_UTILITY} pdal.cpp)
    target_link_libraries(${PDAL_UTILITY} ${PDAL_LINKAGE} ${PDAL_LIB_NAME})
endif()

#------------------------------------------------------------------------------
# Targets installation
#------------------------------------------------------------------------------

install(TARGETS ${PDAL_UTILITIES}
    RUNTIME DESTINATION ${PDAL_BIN_DIR}
    LIBRARY DESTINATION ${PDAL_LIB_DIR}
    ARCHIVE DESTINATION ${PDAL_LIB_DIR})

if(UNIX)

    get_directory_property(PDAL_DEFINITIONS DIRECTORY ${PDAL_SOURCE_DIR}/../src COMPILE_DEFINITIONS)

    set(PDAL_CONFIG_DEFINITIONS "")
    foreach(definition ${PDAL_DEFINITIONS})
        set(PDAL_CONFIG_DEFINITIONS "${PDAL_CONFIG_DEFINITIONS} -D${definition}")
    endforeach()


    set(PKGCFG_PREFIX "${CMAKE_INSTALL_PREFIX}")
    set(PKGCFG_EXEC_PREFIX "${CMAKE_INSTALL_PREFIX}/${PDAL_BIN_DIR}")
    set(PKGCFG_INC_DIR "${CMAKE_INSTALL_PREFIX}/${PDAL_INCLUDE_DIR}")
    set(PKGCFG_LIB_DIR "${CMAKE_INSTALL_PREFIX}/${PDAL_LIB_DIR}")
    set(PKGCFG_REQUIRES  "")
    set(PKGCFG_VERSION ${PDAL_VERSION_STRING})
    set(PKGCFG_LINK_FLAGS "-lpdalcpp")
    set(PKGCFG_COMPILE_FLAGS "")
    if(LIBXML2_FOUND)
      set(PKGCFG_REQUIRES "${PKGCFG_REQUIRES} libxml2-dev")
    endif()
    if(GEOTIFF_FOUND)
      set(PKGCFG_REQUIRES "${PKGCFG_REQUIRES} libgeotiff-dev")
    endif()
    if (GDAL_FOUND)
        set(PKGCFG_REQUIRES "${PKGCFG_REQUIRES} libgdal-dev")
    endif()
    if (LASZIP_FOUND)
        set(PKGCFG_REQUIRES "${PKGCFG_REQUIRES} laszip")
    endif()
    if (P2G_FOUND)
        set(PKGCFG_REQUIRES "${PKGCFG_REQUIRES} points2grid")
    endif()
    if (NITRO_FOUND)
        set(PKGCFG_REQUIRES "${PKGCFG_REQUIRES} nitro-cpp")
    endif()
    if (POSTGRESQL_FOUND)
        set(PKGCFG_REQUIRES "${PKGCFG_REQUIRES} libpq-dev")
    endif()
    if (MSGPACK_FOUND)
        set(PKGCFG_REQUIRES "${PKGCFG_REQUIRES} libmsgpack-dev")
    endif()
    if (SOCI_FOUND)
        set(PKGCFG_REQUIRES "${PKGCFG_REQUIRES} libmsgpack-dev")
    endif()

    set(PKGCFG_REQUIRES "${PKGCFG_REQUIRES} libboost-dev")

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/pdal.pc.in
      ${CMAKE_CURRENT_BINARY_DIR}/pdal.pc @ONLY)

    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pdal.pc
        DESTINATION ${PDAL_PKGCFG_DIR}
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

    # Autoconf compatibility variables to use the same script source.
    set(prefix ${CMAKE_INSTALL_PREFIX})
    set(exec_prefix ${CMAKE_INSTALL_PREFIX}/bin)
    set(libdir ${CMAKE_INSTALL_PREFIX}/lib)



    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/pdal-config.in
      ${CMAKE_CURRENT_BINARY_DIR}/pdal-config @ONLY)

    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pdal-config
      DESTINATION "${CMAKE_INSTALL_PREFIX}/${PDAL_BIN_DIR}"
      PERMISSIONS
      OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

endif()
