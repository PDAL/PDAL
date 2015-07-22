set(INCLUDE_INSTALL_DIR include/ CACHE PATH "include")
set(LIB_INSTALL_DIR lib/ CACHE PATH "lib")
set(SYSCONFIG_INSTALL_DIR etc/pdal/ CACHE PATH "sysconfig")

include(CMakePackageConfigHelpers)

set(PDAL_CONFIG_INCLUDE_DIRS
  "${CMAKE_INSTALL_PREFIX}/include"
  "${Boost_INCLUDE_DIRS}")
set(PDAL_CONFIG_LIBRARY_DIRS
  "${CMAKE_INSTALL_PREFIX}/lib"
  "${Boost_LIBRARY_DIRS}")

configure_package_config_file(
  PDALConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/PDALConfig.cmake
  INSTALL_DESTINATION ${LIB_INSTALL_DIR}/pdal/cmake
  PATH_VARS INCLUDE_INSTALL_DIR SYSCONFIG_INSTALL_DIR)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/PDALConfigVersion.cmake
  VERSION ${PDAL_VERSION_STRING}
  COMPATIBILITY SameMajorVersion)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/PDALConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/PDALConfigVersion.cmake
  DESTINATION ${LIB_INSTALL_DIR}/pdal/cmake)
