#
# Make sure we don't attempt to add a library more than once
#
get_property(EXISTS GLOBAL PROPERTY _UTIL_INCLUDED)
if(EXISTS)
    return()
endif()

add_library(PdalUtil SHARED Bounds.cpp Bounds.hpp)

set_property(GLOBAL PROPERTY _UTIL_INCLUDED TRUE)
