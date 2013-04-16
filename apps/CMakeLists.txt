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

set(PC2PC pc2pc)
set(PCINFO pcinfo)
set(PCPIPELINE pcpipeline)
set(PCQUERY pcquery)
set(PCEQUAL pcequal)

set(COMMON_APP_SOURCES Application.cpp Application.hpp AppSupport.cpp AppSupport.hpp)

set(PDAL_UTILITIES ${PCINFO} )
#    ${PCPIPELINE} ${PCINFO} ${PC2PC} ${PCVIEW})


#------------------------------------------------------------------------------
# Configure build targets
#------------------------------------------------------------------------------

if(WIN32)
    add_definitions("-DPDAL_DLL_EXPORT=1")
endif()

link_directories(${Boost_LIBRARY_DIRS})

# Build pcinfo
if(PCINFO)
    if(LIBXML2_FOUND)
        list(APPEND PDAL_UTILITIES ${PCINFO})
        add_executable(${PCINFO} pcinfo.cpp ${COMMON_APP_SOURCES})
        target_link_libraries(${PCINFO} ${PDAL_LIB_NAME} ${Boost_LIBRARIES})
    endif(LIBXML2_FOUND)
endif(PCINFO)

if(PC2PC)
    list(APPEND PDAL_UTILITIES ${PC2PC})
    add_executable(${PC2PC} pc2pc.cpp ${COMMON_APP_SOURCES})
    target_link_libraries(${PC2PC} ${PDAL_LIB_NAME} ${Boost_LIBRARIES} )
endif()

if(PCPIPELINE)
    list(APPEND PDAL_UTILITIES ${PCPIPELINE})
    add_executable(${PCPIPELINE} pcpipeline.cpp ${COMMON_APP_SOURCES})
    target_link_libraries(${PCPIPELINE} ${PDAL_LIB_NAME} ${Boost_LIBRARIES} )
endif()

if(PCQUERY)
    list(APPEND PDAL_UTILITIES ${PCQUERY})
    add_executable(${PCQUERY} pcquery.cpp ${COMMON_APP_SOURCES})
    target_link_libraries(${PCQUERY} ${PDAL_LIB_NAME} ${Boost_LIBRARIES} )
endif()

if(PCEQUAL)
    list(APPEND PDAL_UTILITIES ${PCEQUAL})
    add_executable(${PCEQUAL} pcequal.cpp ${COMMON_APP_SOURCES})
    target_link_libraries(${PCEQUAL} ${PDAL_LIB_NAME} ${Boost_LIBRARIES} )
endif()

#------------------------------------------------------------------------------
# Targets installation
#------------------------------------------------------------------------------

install(TARGETS ${PDAL_UTILITIES}
    RUNTIME DESTINATION ${PDAL_BIN_DIR}
    LIBRARY DESTINATION ${PDAL_LIB_DIR}
    ARCHIVE DESTINATION ${PDAL_LIB_DIR})

if(UNIX)

  if(WITH_PKGCONFIG)
    
    set(PKGCFG_PREFIX "${CMAKE_INSTALL_PREFIX}")
    set(PKGCFG_INC_DIR "${PDAL_INCLUDE_SUBDIR}")
    set(PKGCFG_LIB_DIR "${PDAL_LIB_SUBDIR}")
    set(PKGCFG_REQUIRES  "")
    set(PKGCFG_VERSION ${VERSION})
    set(PKGCFG_LINK_FLAGS "-lpdal")
    set(PKGCFG_COMPILE_FLAGS "")
    if(LIBXML2_FOUND)
      set(PKGCFG_REQUIRES "${PKGCFG_REQUIRES} libxml-2.0")
    endif()
    if(GEOTIFF_FOUND)
      set(PKGCFG_REQUIRES "${PKGCFG_REQUIRES} geotiff")
    endif()
    #  if(WITH_GDAL)
    #    set(PKGCFG_INC_DIR "${PKGCFG_INC_DIR} ${GDAL_INCLUDE_DIR}")
    #  endif()
    # install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pdal.pc
    #   DESTINATION ${PDAL_LIB_DIR}/pkgconfig
    #   PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
    
  else()

    # Autoconf compatibility variables to use the same script source.
    set(prefix ${CMAKE_INSTALL_PREFIX})
    set(exec_prefix ${CMAKE_INSTALL_PREFIX}/bin)
    set(libdir ${CMAKE_INSTALL_PREFIX}/lib)

    get_directory_property(PDAL_DEFINITIONS DIRECTORY ${PDAL_SOURCE_DIR}/ COMPILE_DEFINITIONS)   

    set(PDAL_CONFIG_DEFINITIONS "")
    foreach(definition ${PDAL_DEFINITIONS})
        set(PDAL_CONFIG_DEFINITIONS "${PDAL_CONFIG_DEFINITIONS} -D${definition}")
    endforeach()

     
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/pdal-config.in
      ${CMAKE_CURRENT_BINARY_DIR}/pdal-config @ONLY)
      
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pdal-config
      DESTINATION bin/
      PERMISSIONS
      OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

  endif()

endif()
