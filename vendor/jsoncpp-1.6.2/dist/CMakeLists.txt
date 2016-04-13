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

if(UNIX)
  add_definitions("-fPIC")
endif()

add_definitions("-DARBITER_EXTERNAL_JSON")


PDAL_ADD_LIBRARY(${PDAL_JSONCPP_LIB_NAME} STATIC "${PDAL_JSONCPP_SOURCES}")

set_target_properties(${PDAL_JSONCPP_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)

set_property(GLOBAL PROPERTY _PDALJSONCPP_INCLUDED TRUE)

