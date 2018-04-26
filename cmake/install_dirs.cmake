# Set the destination directories for installing stuff.
# Sets PDAL_LIB_INSTALL_DIR. Install libraries here.
# Sets PDAL_BIN_INSTALL_DIR. Install binaries here.
# Sets PDAL_INCLUDE_INSTALL_DIR. Install include files here, preferably in a
# subdirectory named after the library in question (e.g.
# "registration/blorgle.h")
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
if (NOT DEFINED PDAL_LIB_INSTALL_DIR)
    if (DEFINED CMAKE_INSTALL_LIBDIR)
        set(PDAL_LIB_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")
    else()
        set(PDAL_LIB_INSTALL_DIR "lib")
    endif()
endif ()
set(PDAL_DOC_INCLUDE_DIR
    "share/doc/${PROJECT_NAME_LOWER}-${PDAL_VERSION_MAJOR}.${PDAL_VERSION_MINOR}")
set(PDAL_BIN_INSTALL_DIR "bin")
set(PDAL_PLUGIN_INSTALL_DIR "share/pdal/plugins")
if(WIN32)
    set(PDALCONFIG_INSTALL_DIR "cmake")
else(WIN32)
    set(PDALCONFIG_INSTALL_DIR
        "share/${PROJECT_NAME_LOWER}-${PDAL_VERSION_MAJOR}.${PDAL_VERSION_MINOR}")
endif(WIN32)
