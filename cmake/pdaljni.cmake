option(WITH_PDAL_JNI
    "Build PDAL JNI Bindings" FALSE)

set(pdal_defines_h_in "${ROOT_DIR}/pdal_defines.h.in")
set(pdal_defines_h "${CMAKE_CURRENT_BINARY_DIR}/include/pdal/pdal_defines.h")
configure_file(${pdal_defines_h_in} ${pdal_defines_h})

if (WITH_PDAL_JNI)
    set(PDAL_BUILD TURE)
    add_subdirectory(${ROOT_DIR}/java/native/src)
endif()
