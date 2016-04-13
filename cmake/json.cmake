get_property(EXISTS GLOBAL PROPERTY _JSONCPP_INCLUDED)
if(EXISTS)
    return()
endif()


find_package(JSONCPP 1.6.2)
set_package_properties(JSONCPP PROPERTIES TYPE OPTIONAL)
mark_as_advanced(CLEAR JSONCPP_INCLUDE_DIR)
mark_as_advanced(CLEAR JSONCPP_LIBRARY)
include_directories(${JSONCPP_INCLUDE_DIR})
set(PDAL_HAVE_JSONCPP 1)

set_property(GLOBAL PROPERTY _JSONCPP_INCLUDED TRUE)
