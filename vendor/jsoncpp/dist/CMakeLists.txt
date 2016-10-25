#
# Make sure we don't attempt to add a library more than once
#
get_property(EXISTS GLOBAL PROPERTY _PDALJSONCPP_INCLUDED)
if (EXISTS)
    return()
endif()

file(GLOB PDAL_JSONCPP_SOURCES
    "jsoncpp.cpp"
)

PDAL_ADD_LIBRARY(${PDAL_JSONCPP_LIB_NAME} STATIC "${PDAL_JSONCPP_SOURCES}")

if (UNIX)
    target_compile_options(${PDAL_JSONCPP_LIB_NAME} PRIVATE -fPIC)
endif()

#
# This forces dll_export on the symbols so that they're available from any
# DLL that the JSON library gets linked with.
#
if (WIN32)
    target_compile_options(${PDAL_JSONCPP_LIB_NAME} PRIVATE -DJSON_DLL_BUILD)
endif()


set_target_properties(${PDAL_JSONCPP_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)

set_property(GLOBAL PROPERTY _PDALJSONCPP_INCLUDED TRUE)

