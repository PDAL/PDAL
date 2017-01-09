option(WITH_PDAL_JNI
    "Build PDAL JNI Bindings" FALSE)

if (WITH_PDAL_JNI)
    set(PDAL_BUILD TURE)
    add_subdirectory(${ROOT_DIR}/java/native/src)
endif()
