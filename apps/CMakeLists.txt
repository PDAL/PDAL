###############################################################################
#
# apps/CMakeLists.txt controls building of libPC utilities 
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
###############################################################################

include_directories(
    .
    ../include)

###############################################################################
# Collect programs to build

set(PC2PC pc2pc)
set(PCINFO pcinfo)

# Set the build type to release if it is not explicitly set by the user and 
# isn't in the cache yet
if (NOT CMAKE_BUILD_TYPE )
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(LIBPC_UTILITIES
    ${PCINFO} ${PC2PC} )

# TODO: Experimental and requires testing --mloskot
# Generate user-specific settings for Visual Studio project
set(VCPROJ_USER_REMOTE_MACHINE_DEBUG ${MACHINE_NAME})
set(VCPROJ_USER_ENVIRONMENT_DEBUG "${ENVIRONMENT_PATH}")

if(MSVC)
    foreach(utility ${LIBPC_UTILITIES})
        set(USER_FILE ${utility}.vcproj.$ENV{USERDOMAIN}.$ENV{USERNAME}.user)
        set(OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/${USER_FILE})
        message(STATUS "Generating ${CMAKE_GENERATOR} user-specific settings in ${USER_FILE}")
        configure_file(${CMAKE_SOURCE_DIR}/cmake/libPC.vcproj.user.template ${OUTPUT_PATH} @ONLY)
    endforeach()
endif()

###############################################################################
# Configure build targets

if(WIN32)
    add_definitions("-DLIBPC_DLL_EXPORT=1")
endif()


set(APPS_CPP_DEPENDENCIES
    ${LIBPC_LIB_NAME}
    ${TIFF_LIBRARY}
    ${GEOTIFF_LIBRARY}
    ${GDAL_LIBRARY}
    ${SPATIALINDEX_LIBRARY}
	${LASZIP_LIBRARY}
    ${Boost_LIBRARIES})

link_directories(${Boost_LIBRARY_DIRS})

# Build pcinfo
if(PCINFO)
    add_executable(${PCINFO} pcinfo.cpp)
    target_link_libraries(${PCINFO} ${APPS_CPP_DEPENDENCIES} )
endif()

if(PC2PC)
    add_executable(${PC2PC} pc2pc.cpp)
    target_link_libraries(${PC2PC} ${APPS_CPP_DEPENDENCIES} )
endif()


###############################################################################
# Targets installation

install(TARGETS ${LIBPC_UTILITIES}
    RUNTIME DESTINATION ${LIBPC_BIN_DIR}
    LIBRARY DESTINATION ${LIBPC_LIB_DIR}
    ARCHIVE DESTINATION ${LIBPC_LIB_DIR})


if(UNIX)

  if(WITH_PKGCONFIG)
    
    set(PKGCFG_PREFIX "${CMAKE_INSTALL_PREFIX}")
    set(PKGCFG_INC_DIR "${LIBPC_INCLUDE_SUBDIR}")
    set(PKGCFG_LIB_DIR "${LIBPC_LIB_SUBDIR}")
    set(PKGCFG_REQUIRES  "")
    set(PKGCFG_VERSION ${VERSION})
    set(PKGCFG_LINK_FLAGS "-llas -llas_c")
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
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/libpc.pc.in
      ${CMAKE_CURRENT_BINARY_DIR}/libpc.pc @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libpc.pc
      DESTINATION ${LIBPC_LIB_DIR}/pkgconfig
      PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
    
  else()

    # Autoconf compatibility variables to use the same script source.
    set(prefix ${CMAKE_INSTALL_PREFIX})
    set(exec_prefix ${CMAKE_INSTALL_PREFIX}/bin)
    set(libdir ${CMAKE_INSTALL_PREFIX}/lib)

    GET_DIRECTORY_PROPERTY(LIBPC_DEFINITIONS DIRECTORY ${libPC_SOURCE_DIR}/ COMPILE_DEFINITIONS)   

    set(LIBPC_CONFIG_DEFINITIONS "")
    foreach(definition ${LIBPC_DEFINITIONS})
        set(LIBPC_CONFIG_DEFINITIONS "${LIBPC_CONFIG_DEFINITIONS} -D${definition}")
    endforeach()

     
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/libpc-config.in
      ${CMAKE_CURRENT_BINARY_DIR}/libpc-config @ONLY)
      
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libpc-config
      DESTINATION bin/
      PERMISSIONS
      OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

  endif()

endif()
