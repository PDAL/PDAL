set(INCLUDE_INSTALL_DIR include/ CACHE PATH "include")
set(LIB_INSTALL_DIR lib/ CACHE PATH "lib")

include(CMakePackageConfigHelpers)

configure_package_config_file(
  PDALConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/PDALConfig.cmake
  INSTALL_DESTINATION ${LIB_INSTALL_DIR}/cmake/PDAL
  PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/PDALConfigVersion.cmake
  VERSION ${PDAL_VERSION_STRING}
  COMPATIBILITY AnyNewerVersion)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/PDALConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/PDALConfigVersion.cmake
  DESTINATION ${LIB_INSTALL_DIR}/cmake/PDAL)
