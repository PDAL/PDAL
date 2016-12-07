get_property(EXISTS GLOBAL PROPERTY _LIBXML2_INCLUDED)
if(EXISTS)
    return()
endif()
#
# libxml2
#

find_package(LibXml2)
set_package_properties(LibXml2 PROPERTIES TYPE OPTIONAL)
mark_as_advanced(CLEAR LIBXML2_INCLUDE_DIR)
mark_as_advanced(CLEAR LIBXML2_LIBRARIES)
if (LIBXML2_FOUND)
    set(PDAL_HAVE_LIBXML2 1)
endif()

set_property(GLOBAL PROPERTY _LIBXML2_INCLUDED TRUE)
