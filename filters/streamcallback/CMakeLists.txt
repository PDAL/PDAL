#
# StreamCallback filter CMake configuration
#

set(srcs StreamCallbackFilter.cpp)

set(incs StreamCallbackFilter.hpp)

PDAL_ADD_DRIVER(filter streamcallback "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
