if (_ICONV_INCLUDED)
    return()
endif()

#
# If included, ICONV become mandatory
#
find_package(ICONV)
set_package_properties(ICONV PROPERTIES TYPE REQUIRED)
if (NOT ICONV_FOUND)
    message(FATAL_ERROR "ICONV library not found but required.")
else()
    mark_as_advanced(CLEAR ICONV_INCLUDE_DIR)
    mark_as_advanced(CLEAR ICONV_LIBRARIES)
    include_directories(${ICONV_INCLUDE_DIR})
endif()
set(_ICONV_INCLUDED TRUE)
